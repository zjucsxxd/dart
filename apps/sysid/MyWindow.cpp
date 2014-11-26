
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
//  std::cout << "Time stepping" << std::endl;
  if(mController)
    mController->update();

//  std::cout << "World stepping" << std::endl;
  mWorld->step();
//  std::cout << "World stepped" << std::endl;
}
