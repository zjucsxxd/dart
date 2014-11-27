
#include "dart/utils/Paths.h"
#include "dart/simulation/World.h"
#include "dart/utils/urdf/DartLoader.h"
#include "dart/utils/SkelParser.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/BoxShape.h"

#include "dart/gui/SimWindow.h"

#include "MyWindow.h"

std::vector<dart::dynamics::BodyNode*> left_bn;
size_t left_dofs;
std::vector<dart::dynamics::BodyNode*> right_bn;
size_t right_dofs;

dart::dynamics::Skeleton* create_ground()
{
  dart::dynamics::Skeleton* ground = new dart::dynamics::Skeleton("ground");

  dart::dynamics::BodyNode* bn = new dart::dynamics::BodyNode("ground_bn");
  bn->setMass(1.0);

  dart::dynamics::Shape* shape = new dart::dynamics::BoxShape(
        Eigen::Vector3d(5,5,0.01));
  shape->setColor(Eigen::Vector3d(0.5, 0.5, 1.0));
  bn->addVisualizationShape(shape);
  bn->addCollisionShape(shape);

  dart::dynamics::Joint* joint = new dart::dynamics::WeldJoint("ground_joint");
  bn->setParentJoint(joint);

  ground->addBodyNode(bn);
  ground->setMobile(false);

  return ground;
}

void stepTowards(const Eigen::Isometry3d& target,
                 const std::vector<dart::dynamics::BodyNode*> bn_list,
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

void balance(dart::dynamics::Skeleton* robot, const Eigen::Vector3d& com_target)
{
  using namespace dart::dynamics;

  const Eigen::Vector3d com = robot->getWorldCOM();
  Eigen::Vector3d err = com_target-com;
  err[1] = 0;

  Eigen::Matrix3d comJ = robot->getWorldCOMJacobian().block<3,3>(0,0);

//  std::cout << "comJ: \n" << comJ << std::endl;

//  Eigen::Vector3d dq = comJ.inverse()*err;
  Eigen::Vector3d dq = comJ.transpose()*err;

//  std::cout << "err: " << err.transpose() << std::endl;
//  std::cout << "q: "
//            << robot->getPosition(0) << "\t"
//            << robot->getPosition(1) << "\t"
//            << robot->getPosition(2) << std::endl;

  robot->setPosition(0, robot->getPosition(0)+dq[0]);
  robot->setPosition(2, robot->getPosition(2)+dq[2]);

//  std::cout << "dq: " << dq.transpose() << std::endl;
//  std::cout << "q: "
//            << robot->getPosition(0) << "\t"
//            << robot->getPosition(1) << "\t"
//            << robot->getPosition(2) << std::endl;
}


void placeFeet(dart::dynamics::Skeleton* robot,
               const Eigen::Isometry3d& left,
               const Eigen::Isometry3d& right,
               const Eigen::Vector3d& com)
{
  for(size_t i=0; i<20; ++i)
  {
    robot->computeForwardKinematics(true, false, false);
    balance(robot, com);
    robot->computeForwardKinematics(true, false, false);
    stepTowards(left, left_bn, left_dofs);
    stepTowards(right, right_bn, right_dofs);
  }
}

std::vector<Eigen::VectorXd> createTrajectory(dart::dynamics::Skeleton* robot)
{
  std::vector<Eigen::VectorXd> trajectory;
  Eigen::VectorXd config = robot->getPositions();

  size_t lsp = robot->getJoint("j_bicep_left")->getIndexInSkeleton(1);
  size_t rsp = robot->getJoint("j_bicep_right")->getIndexInSkeleton(1);

  const Eigen::Isometry3d base_left = left_bn.back()->getTransform();
  const Eigen::Isometry3d base_right = right_bn.back()->getTransform();
  const Eigen::Vector3d base_pelvis = robot->getBodyNode(0)->getTransform().translation();
  const Eigen::Vector3d base_com = robot->getWorldCOM();

  std::cout << base_left.matrix() << std::endl;

  double T = 10;
  for(size_t i=0; i<(size_t)T/robot->getTimeStep(); ++i)
  {
    double t = i*robot->getTimeStep();

    Eigen::Vector3d pelvis_shift = Eigen::Vector3d(0, -0.1*(1-cos(t*2*M_PI/T))/2, 0);
    config.block<3,1>(0,0) = base_pelvis + pelvis_shift;
    Eigen::Isometry3d left_target(base_left); Eigen::Isometry3d right_target(base_right);
    left_target.pretranslate(-pelvis_shift); right_target.pretranslate(-pelvis_shift);
    placeFeet(robot, left_target, right_target, base_com);

    config = robot->getPositions();

    config[lsp] =  M_PI/4*sin(t*2*M_PI/T);
    config[rsp] =  M_PI/4*sin(t*2*M_PI/T);

    trajectory.push_back(config);
  }

  return trajectory;
}



int main(int argc, char* argv[])
{
  using namespace dart::dynamics;
//  dart::simulation::World* world = new dart::simulation::World;

//
//  dart::dynamics::Skeleton* ground = create_ground();
//  world->addSkeleton(ground);

//  dart::utils::DartLoader dl;
//  dl.setPackageDirectory("/home/grey/resources/drchubo");

//  // TODO: Generalize this to take in an argv
//  dart::dynamics::Skeleton* robot =
//      dl.parseSkeleton("/home/grey/resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf");


//  robot->getJoint(0)->setPosition(5, 0.9);
//  robot->computeForwardKinematics(true, true, true);

//  world->addSkeleton(robot);

//  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
////  world->setGravity(Eigen::Vector3d(0, 0, 0));
//  world->setTimeStep(0.001);


  dart::simulation::World* world
      = dart::utils::SkelParser::readWorld(
          DART_DATA_PATH"skel/fullbody1.skel");
  assert(world != NULL);

  world->setGravity(Eigen::Vector3d(0, -9.81, 0));
//  world->setGravity(Eigen::Vector3d(0,0,0));

  std::vector<size_t> genCoordIds;
  genCoordIds.push_back(1);
  genCoordIds.push_back(6);   // left hip
  genCoordIds.push_back(14);   // left knee
  genCoordIds.push_back(17);  // left ankle
  genCoordIds.push_back(9);  // right hip
  genCoordIds.push_back(15);  // right knee
  genCoordIds.push_back(19);  // right ankle
  genCoordIds.push_back(13);  // lower back
  Eigen::VectorXd initConfig(8);
  initConfig << -0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
  world->getSkeleton(1)->setPositionSegment(genCoordIds, initConfig);
  world->getSkeleton(1)->computeForwardKinematics(true, true, false);


  dart::dynamics::Skeleton* robot = world->getSkeleton(1);
  for(size_t j=0; j<robot->getNumBodyNodes(); ++j)
    std::cout << robot->getBodyNode(j)->getName() << std::endl;

  left_bn.push_back(robot->getBodyNode("h_thigh_left"));
  left_bn.push_back(robot->getBodyNode("h_shin_left"));
  left_bn.push_back(robot->getBodyNode("h_heel_left"));
  left_bn.push_back(robot->getBodyNode("h_toe_left"));
  left_dofs = 0;
  for(size_t i=0; i < left_bn.size(); ++i)
    left_dofs += left_bn[i]->getParentJoint()->getNumDofs();

  right_bn.push_back(robot->getBodyNode("h_thigh_right"));
  right_bn.push_back(robot->getBodyNode("h_shin_right"));
  right_bn.push_back(robot->getBodyNode("h_heel_right"));
  right_bn.push_back(robot->getBodyNode("h_toe_right"));
  right_dofs = 0;
  for(size_t i=0; i < right_bn.size(); ++i)
    right_dofs += right_bn[i]->getParentJoint()->getNumDofs();

  const Eigen::VectorXd& config = robot->getPositions();

  Controller* controller = new Controller(robot);
  controller->dt = world->getTimeStep();
  std::cout << "Creating trajectory" << std::endl;
  controller->mDesiredTrajectory = createTrajectory(robot);
  std::cout << "Trajectory created" << std::endl;

  robot->setPositions(config);
  robot->computeForwardKinematics(true, true, true);

  MyWindow window(controller);
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Forward Simulation");
  glutMainLoop();

  delete world;
}
