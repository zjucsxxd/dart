#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <vector>
#include "dart/math/MathTypes.h"

namespace dart {
namespace dynamics {
class Skeleton;
class BodyNode;
} // namespace dynamics
} // namespace dart

class TrajectoryGenerator
{
public:
  TrajectoryGenerator(dart::dynamics::Skeleton* _robot,
                      const std::vector<dart::dynamics::BodyNode*>& _left_leg,
                      const std::vector<dart::dynamics::BodyNode*>& _right_leg);

  void squat(Eigen::VectorXd& config, double time, double depth, double sway);
  void arm_swing(Eigen::VectorXd& config, double time, double amplitude);

  std::vector<Eigen::VectorXd> createTrajectory();

protected:

  double period;

  Eigen::Isometry3d base_left;
  Eigen::Isometry3d base_right;
  Eigen::Vector3d base_pelvis;
  Eigen::Isometry3d base_com;

  std::vector<dart::dynamics::BodyNode*> left_leg;
  size_t left_leg_dofs;
  std::vector<dart::dynamics::BodyNode*> right_leg;
  size_t right_leg_dofs;

  dart::dynamics::Skeleton* robot;

  static void stepTowards(const Eigen::Isometry3d& target,
                          std::vector<dart::dynamics::BodyNode*> bn_list,
                          size_t numDofs);

  void balance(const Eigen::Isometry3d& com_target);

  void placeFeet(const Eigen::Isometry3d& left,
                 const Eigen::Isometry3d& right,
                 const Eigen::Isometry3d& com);

};

#endif // TRAJECTORYGENERATOR_H
