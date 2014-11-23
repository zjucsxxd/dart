
#include "dart/utils/Paths.h"
#include "dart/simulation/World.h"
#include "dart/utils/urdf/DartLoader.h"

#include "dart/gui/SimWindow.h"


int main(int argc, char* argv[])
{
  dart::simulation::World* world = new dart::simulation::World;

  dart::utils::DartLoader dl;
  dart::dynamics::Skeleton* ground =
      dl.parseSkeleton(DART_DATA_PATH"urdf/KR5/ground.urdf");

  // TODO: Generalize this to take in an argv
  dart::dynamics::Skeleton* robot =
      dl.parseSkeleton("/home/grey/resources/drchubo/drchubo_v2/robots/drchubo_v2.urdf");

  world->addSkeleton(ground);
  world->addSkeleton(robot);

  world->setGravity(Eigen::Vector3d(0, 0, -9.81));
  world->setTimeStep(0.001);

  dart::gui::SimWindow window;
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Forward Simulation");
  glutMainLoop();

  delete world;
}
