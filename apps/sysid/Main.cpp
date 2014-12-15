
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
#include "dart/constraint/WeldJointConstraint.h"
#include "dart/constraint/BallJointConstraint.h"
#include "dart/constraint/ConstraintSolver.h"

#include "TrajectoryGenerator.h"

#include "dart/gui/SimWindow.h"

#include "MyWindow.h"

#include "cstdlib"

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

  dart::simulation::World* world = new dart::simulation::World;

  dart::dynamics::Skeleton* ground = create_ground();
  world->addSkeleton(ground);

  dart::utils::DartLoader dl;
  dl.setPackageDirectory("/home/grey/resources/drchubo");

  // TODO: Generalize this to take in an argv
  dart::dynamics::Skeleton* robot =
      dl.parseSkeleton("/home/grey/resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf");


  robot->getJoint(0)->setPosition(5, 1);
  robot->computeForwardKinematics(true, true, true);

  world->addSkeleton(robot);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
//  world->setGravity(Eigen::Vector3d(0, 0, 0));
  world->setTimeStep(0.001);

//  for(size_t i=0; i<robot->getNumBodyNodes(); ++i)
//    std::cout << robot->getBodyNode(i)->getName() << std::endl;

  std::vector<BodyNode*> left_leg_bn;
  left_leg_bn.push_back(robot->getBodyNode("Body_LHY"));
  left_leg_bn.push_back(robot->getBodyNode("Body_LHR"));
  left_leg_bn.push_back(robot->getBodyNode("Body_LHP"));
  left_leg_bn.push_back(robot->getBodyNode("Body_LKP"));
  left_leg_bn.push_back(robot->getBodyNode("Body_LAP"));
  left_leg_bn.push_back(robot->getBodyNode("Body_LAR"));

  std::vector<BodyNode*> right_leg_bn;
  right_leg_bn.push_back(robot->getBodyNode("Body_RHY"));
  right_leg_bn.push_back(robot->getBodyNode("Body_RHR"));
  right_leg_bn.push_back(robot->getBodyNode("Body_RHP"));
  right_leg_bn.push_back(robot->getBodyNode("Body_RKP"));
  right_leg_bn.push_back(robot->getBodyNode("Body_RAP"));
  right_leg_bn.push_back(robot->getBodyNode("Body_RAR"));

  std::vector<BodyNode*> left_arm_bn;
  std::vector<BodyNode*> right_arm_bn;

  robot->getJoint("LHP")->setPosition(0, -20*M_PI/180);
  robot->getJoint("LKP")->setPosition(0,  40*M_PI/180);
  robot->getJoint("LAP")->setPosition(0, -20*M_PI/180);

  robot->getJoint("RHP")->setPosition(0, -20*M_PI/180);
  robot->getJoint("RKP")->setPosition(0,  40*M_PI/180);
  robot->getJoint("RAP")->setPosition(0, -20*M_PI/180);

  const Eigen::VectorXd& config = robot->getPositions();

  robot->computeForwardKinematics(true, false, false);

  Controller* controller = new Controller(robot);
  controller->dt = world->getTimeStep();
  std::cout << "Creating trajectory" << std::endl;
  TrajectoryGenerator generator(robot, left_leg_bn, right_leg_bn, left_arm_bn, right_arm_bn);
  controller->mDesiredTrajectory = generator.createTrajectory();
  std::cout << "Trajectory created" << std::endl;

  robot->setPositions(config);
  robot->computeForwardKinematics(true, true, true);

  MyWindow* window = new MyWindow(controller);
  window->setWorld(world);

  return window;

//  return NULL;
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

  std::vector<dart::dynamics::BodyNode*> left_leg_bn;
  left_leg_bn.push_back(robot->getBodyNode("h_thigh_left"));
  left_leg_bn.push_back(robot->getBodyNode("h_shin_left"));
  left_leg_bn.push_back(robot->getBodyNode("h_heel_left"));

  std::vector<dart::dynamics::BodyNode*> right_leg_bn;
  right_leg_bn.push_back(robot->getBodyNode("h_thigh_right"));
  right_leg_bn.push_back(robot->getBodyNode("h_shin_right"));
  right_leg_bn.push_back(robot->getBodyNode("h_heel_right"));

  std::vector<dart::dynamics::BodyNode*> left_arm_bn;
  left_arm_bn.push_back(robot->getBodyNode("h_bicep_left"));
  left_arm_bn.push_back(robot->getBodyNode("h_forearm_left"));
  left_arm_bn.push_back(robot->getBodyNode("h_hand_left"));

  std::vector<dart::dynamics::BodyNode*> right_arm_bn;
  right_arm_bn.push_back(robot->getBodyNode("h_bicep_right"));
  right_arm_bn.push_back(robot->getBodyNode("h_forearm_right"));
  right_arm_bn.push_back(robot->getBodyNode("h_hand_right"));


  const Eigen::VectorXd& config = robot->getPositions();

  Controller* controller = new Controller(robot);
  controller->dt = world->getTimeStep();
  std::cout << "Creating trajectory" << std::endl;
  TrajectoryGenerator generator(robot, left_leg_bn, right_leg_bn, left_arm_bn, right_arm_bn);
  controller->mDesiredTrajectory = generator.createTrajectory();
  std::cout << "Trajectory created" << std::endl;

  robot->setPositions(config);
  robot->computeForwardKinematics(true, true, true);

  MyWindow* window = new MyWindow(controller);
  window->setWorld(world);

  return window;
}

dart::dynamics::BodyNode* createBaseLink(dart::dynamics::Skeleton* base, bool weld)
{
  using namespace dart::dynamics;
  using namespace dart::constraint;
  using namespace Eigen;

  double xx=1, yy=1, zz=1, xy=0, xz=0, yz=0;
  double mass = 1;
  double h = -1;

  BodyNode* bn = new BodyNode("base_link");
  Joint* fj;
  if(weld)
  {
    fj = new WeldJoint("base");
    h = -h;
  }
  else
  {
    fj = new FreeJoint("base");
  }
  Shape* vis_shape = new BoxShape(fabs(h)/2*Vector3d(1,1,2));
  vis_shape->setOffset(Vector3d(0,0,h/2));
  bn->addVisualizationShape(vis_shape);
  Shape* col_shape = new BoxShape(fabs(h)/2*Vector3d(1,1,2));
  col_shape->setOffset(Vector3d(0,0,h/2));
  bn->addCollisionShape(col_shape);
  bn->setMass(mass);
  bn->setLocalCOM(Vector3d(0,0,h/2));
  bn->setMomentOfInertia(xx, yy, zz, xy, xz, yz);
  bn->setParentJoint(fj);
  base->addBodyNode(bn);

  return bn;
}

template<typename JointType>
dart::dynamics::BodyNode* addLink(dart::dynamics::Skeleton* arm,
                                  dart::dynamics::BodyNode* parent,
                                  const Eigen::Vector3d& dims,
                                  const Eigen::Vector3d& x,
                                  const Eigen::Vector3d& shift)
{
  using namespace dart::dynamics;
  using namespace Eigen;

  double xx=1, yy=1, zz=1, xy=0, xz=0, yz=0;
  double mass = 1;

  Isometry3d offset = Isometry3d::Identity();
  offset.translate(x);

  BodyNode* bn = new BodyNode("link");
  JointType* joint = new JointType;
  joint->setName("joint");
  joint->setTransformFromParentBodyNode(offset);
  Shape* vis_shape = new BoxShape(dims);
  vis_shape->setOffset(shift);
  bn->addVisualizationShape(vis_shape);
  Shape* col_shape = new BoxShape(dims);
  col_shape->setOffset(shift);
  bn->addCollisionShape(col_shape);
  bn->setMass(mass);
  bn->setLocalCOM(shift);
  bn->setMomentOfInertia(xx, yy, zz, xy, xz, yz);
  bn->setParentJoint(joint);
  arm->addBodyNode(bn);

  if(parent)
    parent->addChildBodyNode(bn);

  return bn;
}

dart::dynamics::BodyNode* addBallLink(dart::dynamics::Skeleton* arm,
                                      dart::dynamics::BodyNode* parent)
{
  using namespace dart::dynamics;
  using namespace Eigen;
  double h = -1;

  return addLink<BallJoint>(arm, parent,
                            Vector3d(0.3,0.3,fabs(h)), Vector3d(0,0,h),
                            Vector3d(0,0,h/2));
}

dart::dynamics::BodyNode* addSimpleLink(dart::dynamics::Skeleton* arm,
                                        dart::dynamics::BodyNode* parent,
                                        bool axis_y = true)
{
  using namespace dart::dynamics;
  using namespace Eigen;

  double h = -1;

  BodyNode* bn = addLink<RevoluteJoint>(arm, parent,
                                Vector3d(0.3,0.3,fabs(h)), Vector3d(0,0,h),
                                Vector3d(0,0,h/2));

  Vector3d axis;
  if(axis_y)
    axis = Vector3d::UnitY();
  else
    axis = Vector3d::UnitX();

  dynamic_cast<RevoluteJoint*>(bn->getParentJoint())->setAxis(axis);

  return bn;
}

dart::dynamics::BodyNode* addTangentLink(dart::dynamics::Skeleton* arm,
                                         dart::dynamics::BodyNode* parent)
{
  using namespace dart::dynamics;
  using namespace Eigen;

  double w = 0.3;
  double h = 1;

  BodyNode* bn = addLink<RevoluteJoint>(arm, parent,
                                Vector3d(h,w,w), Vector3d(w/2,0,-h/2),
                                Vector3d(h/2,0,0));

  dynamic_cast<RevoluteJoint*>(bn->getParentJoint())->setAxis(Vector3d(0,0,1));

  return bn;

}

void generateSwings(std::vector<Eigen::VectorXd>& trajectory,
                    dart::dynamics::Skeleton* arm,
                    const Eigen::VectorXi& active,
                    double amplitude, double period)
{
  Eigen::VectorXd config(arm->getNumDofs());
  config.setZero();

  for(size_t k=0; k<(size_t)(period/arm->getTimeStep()); ++k)
  {
    double t = k*arm->getTimeStep();
    for(size_t i=0; i<arm->getNumDofs(); ++i)
    {
      config[i] = active[i]*amplitude*sin(t*2*M_PI/period);

      trajectory.push_back(config);
    }
  }
}

std::vector<Eigen::VectorXd> generateSimpleTrajectory(dart::dynamics::Skeleton* arm)
{
  std::vector<Eigen::VectorXd> trajectory;
  Eigen::VectorXd config(arm->getNumDofs());
  config.setZero();

  double amplitude = 100.0*M_PI/180.0;

  Eigen::VectorXi active(arm->getNumDofs());
  active.setZero();

  for(size_t i=0; i<arm->getNumDofs(); ++i)
  {
    active.setZero();
    active[i] = 1;
    generateSwings(trajectory, arm, active, amplitude, 0.5);
  }

  return trajectory;
}

double getRandomValue(double min, double max)
{
  int rval = rand();
  return (double)(rval%RAND_MAX)/((double)RAND_MAX-1)*(max-min) + min;
}

void mangle_inertias(dart::dynamics::Skeleton* robot)
{
  double m_max = 5;
  double I_max = 5;
  double c_max = 1;
  for(size_t i=0; i<robot->getNumBodyNodes(); ++i)
  {
    dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
    bn->setMass(getRandomValue(0, m_max));
    bn->setLocalCOM(Eigen::Vector3d(getRandomValue(-c_max, c_max),
                                    getRandomValue(-c_max, c_max),
                                    getRandomValue(0, c_max)));
    bn->setMomentOfInertia(getRandomValue(0,I_max),
                           getRandomValue(0,I_max),
                           getRandomValue(0,I_max),
                           0, 0, 0);
  }
}

MyWindow* create_simple_arm()
{
  using namespace dart::dynamics;
  using namespace dart::constraint;
  using namespace Eigen;

  dart::simulation::World* world = new dart::simulation::World;
  world->setGravity(Vector3d(0,0,-9.81));
//  world->setGravity(Vector3d(0,0,0));
  world->setTimeStep(0.001);

  Skeleton* arm = new Skeleton("arm");

//  BodyNode* parent = createBaseLink(arm, false);
//  BodyNode* parent = addBallLink(arm, NULL);
  BodyNode* parent = NULL;
  for(size_t i=0; i<3; ++i)
  {
    parent = addSimpleLink(arm, parent);
  }
//  parent = addTangentLink(arm, arm->getBodyNode(1));
//  parent = addTangentLink(arm, parent);
  parent = addBallLink(arm, parent);

  world->addSkeleton(arm);

  mangle_inertias(arm);

  const Eigen::VectorXd& config = arm->getPositions();

  Controller* controller = new Controller(arm, false);
  controller->dt = world->getTimeStep();
//  std::cout << "Creating trajectory" << std::endl;
  controller->mDesiredTrajectory = generateSimpleTrajectory(arm);
//  std::cout << "Trajectory created" << std::endl;

  arm->setPositions(config);
  arm->computeForwardKinematics(true, true, true);

  MyWindow* window = new MyWindow(controller);
  window->setWorld(world);

  return window;
}

int main(int argc, char* argv[])
{
  srand(time(NULL));
//  MyWindow* window = create_simple_humanoid();
  MyWindow* window = create_simple_arm();
//  MyWindow* window = create_hubo_world();

  glutInit(&argc, argv);
  window->initWindow(640, 480, "Forward Simulation");
  glutMainLoop();
}
