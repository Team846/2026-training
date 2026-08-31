#include "FunkyRobot.h"

#include <frc/DSControlWord.h>
#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "rsighandler.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {}

void FunkyRobot::OnInitialize() {}

void FunkyRobot::OnDisable() {}

void FunkyRobot::InitTeleop() { 
  /*
  Setup ONE triggers so that the command completely runs 100 loops after teleop is enabled,
  stopping after it has run for 100 loops
  and one that runs every 400 loops (not on the first loop though).
  */

  frc2::Trigger... //Finish!
}

void FunkyRobot::InitTest() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  // configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif