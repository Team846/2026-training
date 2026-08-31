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
  frc2::Trigger to_360deg_trigger{
      [&] { return xbox_controller_.GetLeftTriggerAxis() > 0.5; }};
  frc2::Trigger to_0deg_trigger{
      [&] { return xbox_controller_.GetRightTriggerAxis() > 0.5; }};

  to_0deg_trigger.OnTrue(finish this... what does the command take in?);
  to_360deg_trigger.OnTrue(finish this... what does the command take in?);
}

void FunkyRobot::OnPeriodic() {}

void FunkyRobot::InitTest() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  // configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif