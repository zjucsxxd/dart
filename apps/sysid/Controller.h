#ifndef APPS_SYSID_CONTROLLER_H
#define APPS_SYSID_CONTROLLER_H

#include <Eigen/Eigen>

#include "dart/dynamics/Skeleton.h"

class Controller
{
public:

  Controller(dart::dynamics::Skeleton* _robot, double _kp=2, double _kd=1);

  std::vector<Eigen::VectorXd> mDesiredTrajectory;

  const std::vector<Eigen::VectorXd>& getActualTrajectory() const;

  void update();

  double mKp;
  double mKd;

protected:

  size_t step;

  dart::dynamics::Skeleton* mRobot;

  std::vector<Eigen::VectorXd> mActualTrajectory;
  std::vector<Eigen::VectorXd> mActualVelocities;
  std::vector<Eigen::VectorXd> mActualAccelerations;
  std::vector<Eigen::Vector6d> mExternalForces;

};

#endif // APPS_SYSID_CONTROLLER_H
