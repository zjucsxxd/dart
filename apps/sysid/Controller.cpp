#include "Controller.h"

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/WeldJoint.h"

#include <cstdlib>

Controller::Controller(dart::dynamics::Skeleton* _robot, double _kp, double _kd) :
  mKp(_kp),
  mKd(_kd),
  max_torque(1),
  step(0),
  mRobot(_robot),
  calculator(_robot)
{
  srand(time(NULL));
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

void Controller::update()
{
  using namespace dart::dynamics;
  if(step > 0)
  {
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
      const WeldJoint* wj = dynamic_cast<const WeldJoint*>(mRobot->getJoint(i));
      if(wj)
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

//    if(step % 100 == 0)
//    {
//      std::cout << mSampleDynamics[step-1]
//                << "\n________________\n" << std::endl;

//    }

  } // saving data

  for(size_t i=0; i<mRobot->getNumDofs(); ++i)
  {
    int randomVal = rand();
    double value = (double)(randomVal%RAND_MAX)/(RAND_MAX-1)*(2*max_torque) - max_torque;
    mRobot->setForce(i, value);
  }

  ++step;
}

//void Controller::update()
//{
//  std::cout << "Controller update" << std::endl;
//  using namespace dart::dynamics;

//  if(0 > step && step <= mDesiredTrajectory.size())
//  {
//    mActualTrajectory.push_back(mRobot->getPositions());
//    mActualVelocities.push_back(mRobot->getVelocities());
//    mActualAccelerations.push_back(mRobot->getAccelerations());

//    Eigen::Vector6d ext_force;
//    ext_force.setZero();
//    for(size_t i=0, end=mRobot->getNumBodyNodes(); i<end; ++i)
//    {
//      ext_force += mRobot->getBodyNode(i)->getExternalForceGlobal();
//    }
//    mExternalForces.push_back(ext_force);
//  }

//  if(mDesiredTrajectory.size() == 0)
//    return;

//  bool finished = step >= mDesiredTrajectory.size();
//  size_t index = finished? mDesiredTrajectory.size()-1 : step;

//  const Eigen::VectorXd& vels = mRobot->getVelocities();
//  const Eigen::VectorXd& accs = mRobot->getAccelerations();
//  for(size_t j=0, end=mRobot->getNumDofs(); j<end; ++j)
//  {
//    mRobot->setAcceleration(j, 0);
//    mRobot->setVelocity(j, 0);
//  }
//  mRobot->computeInverseDynamics();

//  for(size_t j=0; j<6; ++j)
//    mRobot->setForce(j, 0);

//  double dt = mRobot->getTimeStep();
//  for(size_t j=6, end=mRobot->getNumDofs(); j<end; ++j)
//  {
////    double force = mRobot->getForce(j);
//    double force = 0;
//    double err = mDesiredTrajectory[index][j] - mRobot->getPosition(j);
////    double err = 0;

////    double err_dot = finished? 0 : (index>0?
////            (mDesiredTrajectory[index][j] - mDesiredTrajectory[index-1][j])/dt : 0);
////    err_dot = err_dot - mRobot->getVelocity(j);

//    double err_dot = 0;

//    force += mKp*err + mKd*err_dot;

//    mRobot->setForce(j, force);

////    mRobot->get
//  }

//  mRobot->setVelocities(vels);
//  mRobot->setAccelerations(accs);

//  ++step;

//}
