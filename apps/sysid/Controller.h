#ifndef APPS_SYSID_CONTROLLER_H
#define APPS_SYSID_CONTROLLER_H

#include <Eigen/Eigen>

#include "dart/dynamics/Skeleton.h"
#include "SystemCalculator.h"


class Controller
{
public:

  Controller(dart::dynamics::Skeleton* _robot, double _kp=0.5, double _kd=0);

  std::vector<Eigen::VectorXd> mDesiredTrajectory;

  const std::vector<Eigen::VectorXd>& getActualTrajectory() const;

  void update();

  double mKp;
  double mKd;

  double max_torque;

protected:

  size_t step;

  dart::dynamics::Skeleton* mRobot;

  std::vector<Eigen::VectorXd> mActualTrajectory;
  std::vector<Eigen::VectorXd> mActualVelocities;
  std::vector<Eigen::VectorXd> mActualAccelerations;
  std::vector<Eigen::Vector6d> mExternalForces;

  std::vector<Eigen::Matrix0610d> mSampleA;
  std::vector<Eigen::VectorXd> mSampleForces;
  std::vector<Eigen::MatrixXd> mSampleDynamics;


  SystemCalculator calculator;

};

#endif // APPS_SYSID_CONTROLLER_H
