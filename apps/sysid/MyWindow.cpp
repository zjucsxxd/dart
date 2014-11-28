
#include "MyWindow.h"
#include "dart/simulation/World.h"


MyWindow::MyWindow(Controller *_controller) :
  everythingFinished(false),
  mController(_controller)
{

}

MyWindow::~MyWindow()
{

}

void MyWindow::timeStepping()
{
  if(mController)
    mController->update();

  if(!mController->finished())
    mWorld->step();
  else
  {
    if(everythingFinished)
      return;

    std::cout << "Starting post processing" << std::endl;
    mController->startPostProcessing();
    everythingFinished = true;
  }
}
