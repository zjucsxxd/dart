#include "Controller.h"

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/WeldJoint.h"

#include <cstdlib>
#include <iomanip>

Controller::Controller(dart::dynamics::Skeleton* _robot, bool _floater) :
  max_torque(1),
  step(0),
  mRobot(_robot),
  finish_notice(false),
  calculator(_robot),
  caught_input_nan(false),
  caught_output_nan(false)
{
  floater = _floater;
  srand(time(NULL));

  size_t nDof = mRobot->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);

  mTorques = Eigen::VectorXd::Zero(nDof);

  const double scale = 10;
//  const double scale = 1;
  for(size_t i=0; i<nDof; ++i)
  {
    mKp(i,i) = 400.0*scale;
    mKd(i,i) = 40*scale;
  }

  if(floater)
  {
    for(size_t i=0; i<6; ++i)
    {
      mKp(i, i) = 0;
      mKd(i, i) = 0;
    }
  }

//  const double ankles = 10;
//  std::vector<dart::dynamics::Joint*> feet;
//  feet.push_back(_robot->getJoint("j_heel_left"));
//  feet.push_back(_robot->getJoint("j_heel_right"));
////  feet.push_back(_robot->getJoint("j_toe_left"));
////  feet.push_back(_robot->getJoint("j_toe_right"));

//  for(size_t f=0; f<feet.size(); ++f)
//  {
//    dart::dynamics::Joint* joint = feet[f];
//    for(size_t i=0; i<joint->getNumDofs(); ++i)
//    {
//      size_t index = joint->getIndexInSkeleton(i);
//      mKp(index,index) *= ankles;
//    }
//  }

  mPreOffset = 0.0;
  lap = 0;
}

const std::vector<Eigen::VectorXd>& Controller::getActualTrajectory() const
{
  return mActualTrajectory;
}

static bool descendsFrom(const dart::dynamics::Skeleton* skel,
                         size_t parent,
                         size_t child)
{
  using namespace dart::dynamics;

  const BodyNode* bn = skel->getBodyNode(child);
  const BodyNode* parentBn = skel->getBodyNode(parent);
  while( bn )
  {
    if(bn == parentBn)
      return true;
    bn = bn->getParentBodyNode();
  }

  return false;
}

void Controller::grab_dynamics()
{
  using namespace dart::dynamics;

  size_t n = mRobot->getNumBodyNodes();
  Eigen::MatrixXd sampleDynamics(6*n,10*n);
  sampleDynamics.setZero();
  Eigen::VectorXd sampleForces(6*n);
  sampleForces.setZero();

  mSampleA.clear();
  for(size_t i=0; i<n; ++i)
  {
    mSampleA.push_back(calculator.computeA(i));
    if(dart::math::isNan(mSampleA[i]))
      std::cout << "NaN: " << mRobot->getJoint(i)->getName()
                << "\n" << mSampleA[i]
                << "\n---------------" << std::endl;
  }

  for(size_t i=0; i<n; ++i)
  {
    BodyNode* bn = mRobot->getBodyNode(i);

    for(size_t j=i; j<n; ++j)
    {
      if(!descendsFrom(mRobot, i, j))
        continue;

//      const Eigen::Matrix6d X = calculator.computeSpatialTransform(j, i);
      const Eigen::Matrix6d X = calculator.computeSpatialForceTransform(i, j);
      sampleDynamics.block(6*i, 10*j, 6, 10) = X*mSampleA[j];
    }
    sampleForces.block(6*i, 0, 6, 1) = bn->getBodyForce();
  }

//  for(size_t i=0; i<sampleDynamics.rows(); ++i)
//  {
//    for(size_t j=0; j<sampleDynamics.cols(); ++j)
//      std::cout << std::setw(10) << std::setprecision(3) << sampleDynamics(i,j);
//    std::cout << "\n";
//  }
//  std::cout << std::flush;

  mSampleDynamics.push_back(sampleDynamics);
  mSampleForces.push_back(sampleForces);
}

//void Controller::grab_dynamics()
//{
//  using namespace dart::dynamics;

//  size_t n = mRobot->getNumBodyNodes();
//  size_t dof = mRobot->getNumDofs();
//  Eigen::MatrixXd sampleDynamics(dof,10*n);
//  sampleDynamics.setZero();
////  Eigen::VectorXd sampleForces(dof);
////  sampleForces.setZero();

//  mSampleA.clear();
//  for(size_t i=0; i<n; ++i)
//  {
//    mSampleA.push_back(calculator.computeA(i));
//    if(dart::math::isNan(mSampleA[i]))
//      std::cout << "NaN: " << mRobot->getJoint(i)->getName()
//                << "\n" << mSampleA[i]
//                << "\n---------------" << std::endl;
//  }

//  size_t location = 0;
//  for(size_t i=0; i<n; ++i)
//  {
//    Joint* joint = mRobot->getJoint(i);

//    Eigen::MatrixXd localJ = joint->getLocalJacobian();
//    size_t s = localJ.cols();

//    for(size_t j=i; j<n; ++j)
//    {
//      if(!descendsFrom(mRobot, i, j))
//        continue;

//      const Eigen::Matrix6d X = calculator.computeSpatialTransform(j, i);
//      sampleDynamics.block(location, 10*j, s, 10) = localJ.transpose()*X*mSampleA[j];
//    }
////    sampleForces.block(location, 0, s, 1) = joint->getForces();

//    location += s;
//  }

//  mSampleDynamics.push_back(sampleDynamics);
////  mSampleForces.push_back(sampleForces);
//  mSampleForces.push_back(mTorques);
//}

void Controller::compute_torques(size_t i1, size_t i0)
{
  Eigen::VectorXd pos = mRobot->getPositions();
  Eigen::VectorXd vel = mRobot->getVelocities();
  Eigen::VectorXd acc = mRobot->getAccelerations();

  const Eigen::VectorXd& rPos0 = mDesiredTrajectory[i0];
  const Eigen::VectorXd& rPos1 = mDesiredTrajectory[i1];

  Eigen::VectorXd rVel1 = (rPos1-rPos0)/dt;

  Eigen::VectorXd constrForces = mRobot->getConstraintForces();

  Eigen::MatrixXd invM = (mRobot->getMassMatrix() + mKd*dt).inverse();
  Eigen::VectorXd p = -mKp * (pos + vel*dt - rPos1);
//  Eigen::VectorXd d = -mKd * (vel + acc*dt - rVel1);
  Eigen::VectorXd d = -mKd * (vel - rVel1);
//  Eigen::VectorXd d = -mKd * (vel);
  Eigen::VectorXd qddot = invM*(-mRobot->getCoriolisAndGravityForces()
                                       + p + d + constrForces);

  mTorques = p + d - mKd * qddot * dt;

//  Eigen::Vector3d com = mRobot->getWorldCOM();
//  Eigen::Vector3d cop = mRobot->getBodyNode("h_heel_left")->getTransform()
//      * Eigen::Vector3d(0.05, 0, 0);

//  double offset = com[0] - cop[0];
//  if (offset < 0.1 && offset > 0.0) {
//    double k1 = 200.0;
//    double k2 = 100.0;
//    double kd = 10.0;
//    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
//    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
//    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
//    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
//    mPreOffset = offset;
//  } else if (offset > -0.2 && offset < -0.05) {
//    double k1 = 2000.0;
//    double k2 = 100.0;
//    double kd = 100.0;
//    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
//    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
//    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
//    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
//    mPreOffset = offset;
//  }

  if(floater)
  {
    for(size_t i=0; i<6; ++i)
      mTorques[i] = 0.0;
  }
}

void Controller::update()
{
  using namespace dart::dynamics;

  if(lap >= 2)
    return;

//  if(0 < step && step <= mDesiredTrajectory.size())
//    grab_dynamics();
  if( (0<step) && (step%50==0) )
    grab_dynamics();

  if(mDesiredTrajectory.size() == 0)
    return;

  bool finished = step >= mDesiredTrajectory.size();
  size_t index1 = finished? mDesiredTrajectory.size()-1 : step;
  size_t index0 = finished? mDesiredTrajectory.size()-1 : (index1==0? 0 : step-1);

  if(finished && !finish_notice)
  {
//    std::cout << "Reached end" << std::endl;
    finish_notice = true;
  }

  if(lap==0)
  {
    compute_torques(index1, index0);
    mTorqueHistory.push_back(mTorques);
    mActualTrajectory.push_back(mRobot->getPositions());
  }
  else
  {
    mTorques = mTorqueHistory[step];
    mPredictedTrajectory.push_back(mRobot->getPositions());
  }


  mRobot->setForces(mTorques);

  ++step;
}


void Controller::doPostProcessing()
{
  using namespace dart::dynamics;
  if(lap==0)
  {
    calculator.postProcessing(mRobot, mSampleForces, mSampleDynamics);
//    Eigen::VectorXd parameters = calculator.postProcessing(mRobot, mSampleForces, mSampleDynamics);

//    for(size_t i=0; i<mRobot->getNumBodyNodes(); ++i)
//    {
//      BodyNode* bn = mRobot->getBodyNode(i);
//      double mass = parameters[10*i];
//      bn->setMass(mass);
//      if(mass)
//        bn->setLocalCOM(parameters.block<3,1>(10*i+1,0)/mass);
//      else
//        bn->setLocalCOM(Eigen::Vector3d::Zero());
//      bn->setMomentOfInertia(parameters[10*i+4], parameters[10*i+7], parameters[10*i+9],
//                             parameters[10*i+5], parameters[10*i+6], parameters[10*i+8]);
//    }

    return;
  }

  if(mActualTrajectory.size() != mPredictedTrajectory.size())
    std::cout << "Size difference between actual and predicted waypoints: "
              << mActualTrajectory.size() << " | " << mPredictedTrajectory.size()
              << std::endl;
  size_t N = std::min(mActualTrajectory.size(), mPredictedTrajectory.size());

  double diff = 0;
  for(size_t i=0; i<N; ++i)
  {
    diff += (mActualTrajectory[i]-mPredictedTrajectory[i]).dot(
        (mActualTrajectory[i]-mPredictedTrajectory[i]));
  }

  diff = sqrt(diff);
  std::cout << "Error between trajectories: " << diff << std::endl;
}

void Controller::reset()
{
  step = 0;
  finish_notice = false;

  mSampleA.clear();
  mSampleForces.clear();
  mSampleDynamics.clear();
}


