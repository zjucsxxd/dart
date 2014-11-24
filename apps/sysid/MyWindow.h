#ifndef APPS_SYSID_MYWINDOW_H_
#define APPS_SYSID_MYWINDOW_H_

#include "dart/gui/SimWindow.h"
#include "Controller.h"


class MyWindow : public dart::gui::SimWindow
{
public:

  MyWindow(Controller* _controller);

  virtual ~MyWindow();

  virtual void timeStepping();


protected:

  Controller* mController;


};


#endif // APPS_SYSID_MYWINDOW_H_
