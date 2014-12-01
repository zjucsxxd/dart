#include "TrajectoryGenerator.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"

TrajectoryGenerator::TrajectoryGenerator(dart::dynamics::Skeleton* _robot,
                                         const std::vector<dart::dynamics::BodyNode*>& _left_leg,
                                         const std::vector<dart::dynamics::BodyNode*>& _right_leg,
                                         const std::vector<dart::dynamics::BodyNode*>& _left_arm,
                                         const std::vector<dart::dynamics::BodyNode*>& _right_arm) :
  left_leg(_left_leg),
  right_leg(_right_leg),
  left_arm(_left_arm),
  right_arm(_right_arm),
  robot(_robot)
{
  base_left = _left_leg.back()->getTransform();
  base_right = _right_leg.back()->getTransform();
  base_pelvis = robot->getBodyNode(0)->getTransform().translation();
  Eigen::Vector3d base_com_x = robot->getWorldCOM();
  base_com = Eigen::Isometry3d::Identity();
  base_com.translate(base_com_x);
  base_com.rotate(robot->getBodyNode(0)->getTransform().rotation());

  left_leg_dofs = 0;
  for(size_t i=0; i<left_leg.size(); ++i)
    left_leg_dofs += left_leg[i]->getParentJoint()->getNumDofs();

  right_leg_dofs = 0;
  for(size_t i=0; i<right_leg.size(); ++i)
    right_leg_dofs += right_leg[i]->getParentJoint()->getNumDofs();

  left_arm_dofs = 0;
  for(size_t i=0; i<left_arm.size(); ++i)
    left_arm_dofs += left_arm[i]->getParentJoint()->getNumDofs();

  right_arm_dofs = 0;
  for(size_t i=0; i<right_arm.size(); ++i)
    right_arm_dofs += right_arm[i]->getParentJoint()->getNumDofs();
}


void TrajectoryGenerator::stepTowards(const Eigen::Isometry3d& target,
                 std::vector<dart::dynamics::BodyNode*> bn_list,
                 size_t numDofs)
{
  using namespace dart::dynamics;
  dart::math::Jacobian J(6, numDofs);
  size_t count = 0;

  BodyNode* last = bn_list.back();

  const Eigen::Vector3d& offset = last->getTransform().inverse()*target.translation();
  J.block(0, 0, 3, numDofs) = last->getBodyLinearJacobian(offset).block(0, 6, 3, numDofs);
  J.block(3, 0, 3, numDofs) = last->getBodyAngularJacobian().block(0, 6, 3, numDofs);

  Eigen::Vector6d error;
  error.block<3,1>(0,0) = target.translation() - last->getTransform().translation();
  const Eigen::Matrix3d& rot = target.rotation()*last->getTransform().rotation().transpose();
  error[3] =  atan2(rot(2,1), rot(2,2));
  error[4] = -asin(rot(2,0));
  error[5] =  atan2(rot(1,0), rot(0,0));

  double damp = 0.05;
  Eigen::VectorXd dq = J.transpose()*(J*J.transpose()+
              damp*damp*Eigen::MatrixXd::Identity(J.rows(),J.rows())).inverse()*error;

  count = 0;
  for(size_t i=0; i<bn_list.size(); ++i)
  {
    BodyNode* bn = bn_list[i];
    Joint* joint = bn->getParentJoint();
    for(size_t j=0; j<joint->getNumDofs(); ++j)
    {
      joint->setPosition(j, joint->getPosition(j)+dq[count]);
      ++count;
    }
  }
}

void TrajectoryGenerator::balance(const Eigen::Isometry3d& com_target)
{
  using namespace dart::dynamics;

  const Eigen::Vector3d com = robot->getWorldCOM();
  Eigen::Vector6d err;
  err.block<3,1>(0,0) = com_target.translation()-com;
  err[1] = 0;
  err[2] = 0;

  const Eigen::Matrix3d& rot = com_target.rotation()*
      robot->getBodyNode(0)->getTransform().rotation().transpose();
  err[3] =  atan2(rot(2,1), rot(2,2));
  err[4] = -asin(rot(2,0));
  err[5] =  atan2(rot(1,0), rot(0,0));

  Eigen::Matrix3d comJ = robot->getWorldCOMJacobian().block<3,3>(0,0);

//  std::cout << "comJ: \n" << comJ << std::endl;

//  Eigen::Vector3d dq = comJ.inverse()*err;
  Eigen::Vector3d dq = comJ.transpose()*err.block<3,1>(0,0);

//  std::cout << "err: " << err.transpose() << std::endl;
//  std::cout << "q: "
//            << robot->getPosition(0) << "\t"
//            << robot->getPosition(1) << "\t"
//            << robot->getPosition(2) << std::endl;

  robot->setPosition(0, robot->getPosition(0)+dq[0]);
  robot->setPosition(2, robot->getPosition(2)+dq[2]);

  Eigen::Matrix3d bodyJ = robot->getBodyNode(0)->getBodyAngularJacobian().block<3,3>(0,0);
//  std::cout << "J:\n" << bodyJ << std::endl;
  dq = bodyJ.inverse()*err.block<3,1>(0,0);

  robot->setPosition(3, robot->getPosition(3)+dq[0]);
  robot->setPosition(4, robot->getPosition(4)+dq[1]);
  robot->setPosition(5, robot->getPosition(5)+dq[2]);

//  std::cout << "dq: " << dq.transpose() << std::endl;
//  std::cout << "q: "
//            << robot->getPosition(0) << "\t"
//            << robot->getPosition(1) << "\t"
//            << robot->getPosition(2) << std::endl;
}


void TrajectoryGenerator::placeFeet(
               const Eigen::Isometry3d& left,
               const Eigen::Isometry3d& right,
               const Eigen::Isometry3d& com)
{
  for(size_t i=0; i<20; ++i)
  {
    robot->computeForwardKinematics(true, false, false);
    balance(com);
    robot->computeForwardKinematics(true, false, false);
    stepTowards(left, left_leg, left_leg_dofs);
    stepTowards(right, right_leg, right_leg_dofs);
  }
}

void TrajectoryGenerator::squat(Eigen::VectorXd &config, double time, double depth, double sway)
{
  Eigen::Vector3d pelvis_shift = Eigen::Vector3d(0, -depth, sway)*(1-cos(time*2*M_PI/period))/2;
  config.block<3,1>(0,0) = base_pelvis + pelvis_shift;
  Eigen::Isometry3d left_target(base_left); Eigen::Isometry3d right_target(base_right);
  left_target.pretranslate(-pelvis_shift); right_target.pretranslate(-pelvis_shift);
  placeFeet(left_target, right_target, base_com);

  config = robot->getPositions();
}

void TrajectoryGenerator::arm_swing(Eigen::VectorXd &config, double time, double amplitude)
{
  using namespace dart::dynamics;
//  size_t lsp = robot->getJoint("j_bicep_left")->getIndexInSkeleton(1);
//  size_t rsp = robot->getJoint("j_bicep_right")->getIndexInSkeleton(1);

  size_t lsp = robot->getJoint("LSP")->getIndexInSkeleton(0);
  size_t rsp = robot->getJoint("RSP")->getIndexInSkeleton(0);

  config[lsp] = amplitude*sin(time*2*M_PI/period);
  config[rsp] = -amplitude*sin(time*2*M_PI/period);

//  for(size_t i=0; i<left_arm.size(); ++i)
//  {
//    Joint* joint = left_arm[i]->getParentJoint();
//    for(size_t j=0; j<joint->getNumDofs(); ++j)
//    {
//      size_t index = joint->getIndexInSkeleton(j);
//      config[index] = amplitude*sin(time*2*M_PI/period);
//    }
//  }

//  for(size_t i=0; i<right_arm.size(); ++i)
//  {
//    Joint* joint = right_arm[i]->getParentJoint();
//    for(size_t j=0; j<joint->getNumDofs(); ++j)
//    {
//      size_t index = joint->getIndexInSkeleton(j);
//      config[index] = -amplitude*sin(time*2*M_PI/period);
//    }
//  }

  robot->setPositions(config);
}

std::vector<Eigen::VectorXd> TrajectoryGenerator::createTrajectory()
{
  std::vector<Eigen::VectorXd> trajectory;
  Eigen::VectorXd config = robot->getPositions();

  period = 10;
  for(size_t i=0; i<(size_t)period/robot->getTimeStep(); ++i)
  {
    double t = i*robot->getTimeStep();

    arm_swing(config, t, M_PI/4);

//    squat(config, t, 0.3, 0.0);
    squat(config, t, 0.0, -0.3);

    trajectory.push_back(config);
  }

  return trajectory;
}
