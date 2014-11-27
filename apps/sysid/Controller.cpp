#include "Controller.h"

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/WeldJoint.h"

#include <cstdlib>

Controller::Controller(dart::dynamics::Skeleton* _robot) :
  max_torque(1),
  step(0),
  mRobot(_robot),
  finish_notice(false),
  calculator(_robot),
  caught_input_nan(false),
  caught_output_nan(false)
{
  srand(time(NULL));

  size_t nDof = mRobot->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);

  mTorques = Eigen::VectorXd::Zero(nDof);

  for(size_t i=0; i<6; ++i)
  {
    mKp(i, i) = 0;
    mKd(i, i) = 0;
  }

  for(size_t i=6; i<nDof; ++i)
  {
    mKp(i,i) = 400.0;
    mKd(i,i) = 40;
  }

  mPreOffset = 0.0;
}

const std::vector<Eigen::VectorXd>& Controller::getActualTrajectory() const
{
  return mActualTrajectory;
}

static bool getAxis(const dart::dynamics::Joint* joint, Eigen::Vector6d& axis)
{
  using namespace dart::dynamics;

  const FreeJoint* fj = dynamic_cast<const FreeJoint*>(joint);
  if(fj)
    return false;

  const RevoluteJoint* rj = dynamic_cast<const RevoluteJoint*>(joint);
  if(rj)
  {
    axis.block<3,1>(0,0) = rj->getAxis();
    axis.block<3,1>(3,0).setZero();
    return true;
  }

  const PrismaticJoint* pj = dynamic_cast<const PrismaticJoint*>(joint);
  if(pj)
  {
    axis.block<3,1>(0,0).setZero();
    axis.block<3,1>(3,0) = pj->getAxis();
    return true;
  }

  return false;
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
  Eigen::MatrixXd sampleDynamics(n+5,10*n);
  sampleDynamics.setZero();
  Eigen::VectorXd sampleForces(n+5);
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
    Eigen::Vector6d axis;
    bool useAxis = getAxis(mRobot->getJoint(i), axis);
    const ZeroDofJoint* zj = dynamic_cast<const ZeroDofJoint*>(mRobot->getJoint(i));
    if(zj)
      continue;

    if(useAxis)
    {
      sampleForces[5+i] = mRobot->getJoint(i)->getForce(0);
//        sampleForces[5+i] = mRobot->getJoint(i)->getForces();
    }
    else
    {
      sampleForces.block<6,1>(0,0) = mRobot->getJoint(i)->getForces();
    }

    for(size_t j=i; j<n; ++j)
    {
      if(!descendsFrom(mRobot, i, j))
        continue;

      const Eigen::Matrix6d X = calculator.computeSpatialTransform(j, i);
      if(useAxis)
      {
        sampleDynamics.block<1,10>(i+5,10*j) = axis.transpose()*X*mSampleA[j];
      }
      else
      {
        sampleDynamics.block<6,10>(0,10*j) = X*mSampleA[j];
      }
    }
  }

  mSampleDynamics.push_back(sampleDynamics);
  mSampleForces.push_back(sampleForces);
}

void Controller::compute_torques(size_t i1, size_t i0)
{
  Eigen::VectorXd pos = mRobot->getPositions();
  Eigen::VectorXd vel = mRobot->getVelocities();

  Eigen::VectorXd constrForces = mRobot->getConstraintForces();

  Eigen::MatrixXd invM = (mRobot->getMassMatrix() + mKd*dt).inverse();
  Eigen::VectorXd p = -mKp * (pos + vel*dt - mDesiredTrajectory[i1]);
  Eigen::VectorXd d = -mKd*vel;
  Eigen::VectorXd qddot = invM*(-mRobot->getCoriolisAndGravityForces()
                                       + p + d + constrForces);

  mTorques = p + d - mKd * qddot * dt;

  Eigen::Vector3d com = mRobot->getWorldCOM();
  Eigen::Vector3d cop = mRobot->getBodyNode("h_heel_left")->getTransform()
      * Eigen::Vector3d(0.05, 0, 0);

  double offset = com[0] - cop[0];
  if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }

  for(size_t i=0; i<6; ++i)
    mTorques[i] = 0.0;
}

void Controller::update()
{
  using namespace dart::dynamics;

  if(0 > step && step <= mDesiredTrajectory.size())
    grab_dynamics();

  if(mDesiredTrajectory.size() == 0)
    return;

  bool finished = step >= mDesiredTrajectory.size();
  size_t index1 = finished? mDesiredTrajectory.size()-1 : step;
  size_t index0 = finished? mDesiredTrajectory.size()-1 : (index1==0? 0 : step-1);

  if(finished && !finish_notice)
  {
    std::cout << "Reached end" << std::endl;
    finish_notice = true;
  }

  compute_torques(index1, index0);

  mRobot->setForces(mTorques);

  ++step;
}
