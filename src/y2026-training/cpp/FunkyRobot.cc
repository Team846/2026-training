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
  frc2::Trigger intake_trigger{
      [&] { return xbox_controller_.GetBButton(); }};
  intake_trigger.WhileTrue(IntakeCommand{container_, 2000_deg_per_s}.ToPtr());
}

void FunkyRobot::OnPeriodic() {}

void FunkyRobot::InitTest() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  // configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif