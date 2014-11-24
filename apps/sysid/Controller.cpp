#include "Controller.h"

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"

Controller::Controller(dart::dynamics::Skeleton* _robot, double _kp, double _kd) :
  mKp(_kp),
  mKd(_kd),
  step(0),
  mRobot(_robot)
{

}

const std::vector<Eigen::VectorXd>& Controller::getActualTrajectory() const
{
  return mActualTrajectory;
}

void Controller::update()
{
  std::cout << "Controller update" << std::endl;
  using namespace dart::dynamics;

  if(0 > step && step <= mDesiredTrajectory.size())
  {
    mActualTrajectory.push_back(mRobot->getPositions());
    mActualVelocities.push_back(mRobot->getVelocities());
    mActualAccelerations.push_back(mRobot->getAccelerations());

    Eigen::Vector6d ext_force;
    ext_force.setZero();
    for(size_t i=0, end=mRobot->getNumBodyNodes(); i<end; ++i)
    {
      ext_force += mRobot->getBodyNode(i)->getExternalForceGlobal();
    }
    mExternalForces.push_back(ext_force);
  }

  if(mDesiredTrajectory.size() == 0)
    return;

  bool finished = step >= mDesiredTrajectory.size();
  size_t index = finished? mDesiredTrajectory.size()-1 : step;


  for(size_t j=0; j<6; ++j)
    mRobot->setForce(j, 0);

  double dt = mRobot->getTimeStep();
  for(size_t j=6, end=mRobot->getNumDofs(); j<end; ++j)
  {
    double err = mDesiredTrajectory[index][j] - mRobot->getPosition(j);
//    double err = 0;

//    double err_dot = finished? 0 : (index>0?
//            (mDesiredTrajectory[index][j] - mDesiredTrajectory[index-1][j])/dt : 0);
//    err_dot = err_dot - mRobot->getVelocity(j);

    double err_dot = 0;

    mRobot->setForce(j, mKp*err + mKd*err_dot);

//    mRobot->get
  }


  ++step;

}
