
#include "dart/utils/Paths.h"
#include "dart/simulation/World.h"
#include "dart/utils/urdf/DartLoader.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/BoxShape.h"

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

std::vector<Eigen::VectorXd> createTrajectory(dart::dynamics::Skeleton* _robot)
{
  std::vector<Eigen::VectorXd> trajectory;
  Eigen::VectorXd config = _robot->getPositions();

  size_t rsp = _robot->getJoint("RSP")->getIndexInSkeleton(0);

  double T = 10;
  for(size_t i=0; i<(size_t)T/_robot->getTimeStep(); ++i)
  {
    double t = i*_robot->getTimeStep();

    config[rsp] = M_PI/4*sin(t);

    trajectory.push_back(config);
  }

  return trajectory;
}



int main(int argc, char* argv[])
{
  dart::simulation::World* world = new dart::simulation::World;

  dart::dynamics::Skeleton* ground = create_ground();

  dart::utils::DartLoader dl;
  dl.setPackageDirectory("/home/grey/resources/drchubo");

  // TODO: Generalize this to take in an argv
  dart::dynamics::Skeleton* robot =
      dl.parseSkeleton("/home/grey/resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf");

  for(size_t j=0; j<robot->getNumBodyNodes(); ++j)
    std::cout << robot->getJoint(j)->getName() << std::endl;

  robot->getJoint(0)->setPosition(5, 1);
  robot->computeForwardKinematics(true, false, false);

  world->addSkeleton(ground);
  world->addSkeleton(robot);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(0.001);


  Controller* controller = new Controller(robot);
  controller->mDesiredTrajectory = createTrajectory(robot);
  MyWindow window(controller);
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Forward Simulation");
  glutMainLoop();

  delete world;
}
