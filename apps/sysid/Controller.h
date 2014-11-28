#ifndef APPS_SYSID_CONTROLLER_H
#define APPS_SYSID_CONTROLLER_H

#include <Eigen/Eigen>

#include "dart/dynamics/Skeleton.h"
#include "SystemCalculator.h"


class Controller
{
public:

  Controller(dart::dynamics::Skeleton* _robot);

  std::vector<Eigen::VectorXd> mDesiredTrajectory;

  const std::vector<Eigen::VectorXd>& getActualTrajectory() const;

  void grab_dynamics();
  void compute_torques(size_t i1, size_t i0);
  void update();

  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  Eigen::VectorXd mTorques;
  double mPreOffset;

  double max_torque;
  double dt;

  inline bool finished() const { return finish_notice; }

  void startPostProcessing();

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

  bool finish_notice;

  SystemCalculator calculator;

  bool caught_input_nan;
  bool caught_output_nan;

};

#endif // APPS_SYSID_CONTROLLER_H
