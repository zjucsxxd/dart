
#include "MyWindow.h"
#include "dart/simulation/World.h"


MyWindow::MyWindow(Controller *_controller) :
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
    if(mController->lap>=2)
      return;

//    std::cout << "Starting post processing (lap " << mController->lap << ")" << std::endl;
    mController->doPostProcessing();

    mController->lap = 2;
    return;


//    mController->reset();

//    ++mController->lap;
  }
}
