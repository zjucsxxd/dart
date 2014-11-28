
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
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/RevoluteJoint.h"

#include "TrajectoryGenerator.h"

#include "dart/gui/SimWindow.h"

#include "MyWindow.h"

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

MyWindow* create_hubo_world()
{
  using namespace dart::dynamics;

//  dart::simulation::World* world = new dart::simulation::World;

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

//  const Eigen::VectorXd& config = robot->getPositions();

//  Controller* controller = new Controller(robot);
//  controller->dt = world->getTimeStep();
//  std::cout << "Creating trajectory" << std::endl;
//  TrajectoryGenerator generator(robot, left_bn, right_bn);
//  controller->mDesiredTrajectory = generator.createTrajectory();
//  std::cout << "Trajectory created" << std::endl;

//  robot->setPositions(config);
//  robot->computeForwardKinematics(true, true, true);

//  MyWindow* window = new MyWindow(controller);
//  window->setWorld(world);

//  return window;

  return NULL;
}

MyWindow* create_simple_humanoid()
{
  using namespace dart::dynamics;

  dart::simulation::World* world
      = dart::utils::SkelParser::readWorld(
          DART_DATA_PATH"skel/fullbody1.skel");
  assert(world != NULL);

  dart::dynamics::Skeleton* robot = world->getSkeleton(1);
  BodyNode* heel = robot->getBodyNode("h_heel_left");
  Joint* oldJoint = heel->getParentJoint();
  heel->setParentJoint(new dart::dynamics::BallJoint("j_heel_left"));
  Joint* newJoint = heel->getParentJoint();
  newJoint->setTransformFromParentBodyNode(oldJoint->getTransformFromParentBodyNode());
  newJoint->setTransformFromChildBodyNode(oldJoint->getTransformFromChildBodyNode());


  heel = robot->getBodyNode("h_heel_right");
  heel->setParentJoint(new dart::dynamics::BallJoint("j_heel_right"));
  newJoint = heel->getParentJoint();
  newJoint->setTransformFromParentBodyNode(oldJoint->getTransformFromParentBodyNode());
  newJoint->setTransformFromChildBodyNode(oldJoint->getTransformFromChildBodyNode());

//  robot->init(world->getTimeStep(), world->getGravity());
  world->removeSkeleton(robot, false);
  world->addSkeleton(robot);

  world->setGravity(Eigen::Vector3d(0, -9.81, 0));
//  world->setGravity(Eigen::Vector3d(0,0,0));

  std::vector<size_t> genCoordIds;
  genCoordIds.push_back(1);
  genCoordIds.push_back(6);   // left hip
  genCoordIds.push_back(14);   // left knee
  genCoordIds.push_back(19);  // left ankle
  genCoordIds.push_back(9);  // right hip
  genCoordIds.push_back(15);  // right knee
  genCoordIds.push_back(22);  // right ankle
  genCoordIds.push_back(13);  // lower back
  Eigen::VectorXd initConfig(8);
  initConfig << -0.2, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
  robot->setPositionSegment(genCoordIds, initConfig);
  robot->computeForwardKinematics(true, true, false);

  std::cout << "BodyNode count: " << robot->getNumBodyNodes() << std::endl;
  std::cout << "Dof count: " << robot->getNumDofs() << std::endl;
  for(size_t j=0; j<robot->getNumBodyNodes(); ++j)
  {
    std::cout << robot->getBodyNode(j)->getName() << ": ";
    for(size_t d=0; d<robot->getJoint(j)->getNumDofs(); ++d)
      std::cout << robot->getJoint(j)->getIndexInSkeleton(d) << ", ";
    std::cout << "(" << robot->getJoint(j)->getNumDofs() << ")" << std::endl;
  }

  std::vector<dart::dynamics::BodyNode*> left_bn;
  left_bn.push_back(robot->getBodyNode("h_thigh_left"));
  left_bn.push_back(robot->getBodyNode("h_shin_left"));
  left_bn.push_back(robot->getBodyNode("h_heel_left"));

  std::vector<dart::dynamics::BodyNode*> right_bn;
  right_bn.push_back(robot->getBodyNode("h_thigh_right"));
  right_bn.push_back(robot->getBodyNode("h_shin_right"));
  right_bn.push_back(robot->getBodyNode("h_heel_right"));

  const Eigen::VectorXd& config = robot->getPositions();

  Controller* controller = new Controller(robot);
  controller->dt = world->getTimeStep();
  std::cout << "Creating trajectory" << std::endl;
  TrajectoryGenerator generator(robot, left_bn, right_bn);
  controller->mDesiredTrajectory = generator.createTrajectory();
  std::cout << "Trajectory created" << std::endl;

  robot->setPositions(config);
  robot->computeForwardKinematics(true, true, true);

  MyWindow* window = new MyWindow(controller);
  window->setWorld(world);

  return window;
}

dart::dynamics::BodyNode* addSimpleLink(dart::dynamics::Skeleton* arm,
                                        dart::dynamics::BodyNode* parent)
{
  using namespace dart::dynamics;
  using namespace Eigen;

  double xx=1, yy=1, zz=1, xy=0, xz=0, yz=0;
  double mass = 1;
  double h = -1;

  Eigen::Isometry3d offset(Eigen::Isometry3d::Identity());
  offset.translate(Vector3d(0,0,h));

  BodyNode* bn = new BodyNode("link");
  RevoluteJoint* joint = new RevoluteJoint(Vector3d(0,1,0), "joint");
  joint->setTransformFromParentBodyNode(offset);
  Shape* vis_shape = new BoxShape(Vector3d(0.3,0.3,fabs(h)));
  vis_shape->setOffset(Vector3d(0,0,h/2));
  bn->addVisualizationShape(vis_shape);
  Shape* col_shape = new BoxShape(Vector3d(0.3,0.3,fabs(h)));
  col_shape->setOffset(Vector3d(0,0,h/2));
  bn->addCollisionShape(col_shape);
  bn->setMass(mass);
  bn->setLocalCOM(Vector3d(0,0,h));
  bn->setMomentOfInertia(xx, yy, zz, xy, xz, yz);
  bn->setParentJoint(joint);
  arm->addBodyNode(bn);

  if(parent)
    parent->addChildBodyNode(bn);

  return bn;
}

std::vector<Eigen::VectorXd> generateSimpleTrajectory(dart::dynamics::Skeleton* arm)
{
  std::vector<Eigen::VectorXd> trajectory;
  Eigen::VectorXd config(arm->getNumDofs());
  config.setZero();

  double amplitude = 120.0*M_PI/180.0;
  double period = 2;
  for(size_t i=0; i<arm->getNumDofs(); ++i)
  {
    for(size_t k=0; k<(size_t)period/arm->getTimeStep(); ++k)
    {
      double t = k*arm->getTimeStep();

      config[i] = amplitude*sin(t*2*M_PI/period);

      trajectory.push_back(config);
    }

    config.setZero();
  }

  return trajectory;
}

MyWindow* create_simple_arm()
{
  using namespace dart::dynamics;
  using namespace Eigen;

  dart::simulation::World* world = new dart::simulation::World;
  world->setGravity(Vector3d(0,0,-9.81));
//  world->setGravity(Vector3d(0,0,0));
  world->setTimeStep(0.001);

  Skeleton* arm = new Skeleton("arm");

  BodyNode* parent = NULL;
  for(size_t i=0; i<3; ++i)
  {
    parent = addSimpleLink(arm, parent);
  }

  for(size_t i=0; i<arm->getNumBodyNodes(); ++i)
  {
    std::cout << arm->getJoint(i)->getName() << " -> "
              << arm->getBodyNode(i)->getName() << std::endl;
  }

  world->addSkeleton(arm);

  const Eigen::VectorXd& config = arm->getPositions();

  Controller* controller = new Controller(arm);
  controller->floater = false;
  controller->dt = world->getTimeStep();
  std::cout << "Creating trajectory" << std::endl;
  controller->mDesiredTrajectory = generateSimpleTrajectory(arm);
  std::cout << "Trajectory created" << std::endl;

  arm->setPositions(config);
  arm->computeForwardKinematics(true, true, true);

  MyWindow* window = new MyWindow(controller);
  window->setWorld(world);

  return window;
}

int main(int argc, char* argv[])
{
  MyWindow* window = create_simple_humanoid();
//  MyWindow* window = create_simple_arm();

  glutInit(&argc, argv);
  window->initWindow(640, 480, "Forward Simulation");
  glutMainLoop();
}
