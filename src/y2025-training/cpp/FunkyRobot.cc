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
  Setup ONE trigger so that the command completely runs 100 loops after teleop is enabled,
  stopping after it has run for 100 loops
  and one that runs every 400 loops (not on the first loop though).
  */

  frc2::Trigger initial_led_trigger{[&] {return 100 <= teleopLoops && teleopLoops < 200;}};
  initial_led_trigger.WhileTrue(LEDsCommand{container_}.ToPtr());
  frc2::Trigger general_led_trigger{[&] {return teleopLoops % 400 == 0 && teleopLoops > 1;}};
  general_led_trigger.OnTrue(LEDsCommand{container_}.ToPtr());
}

void FunkyRobot::OnPeriodic() {
  teleopLoops++;
}

void FunkyRobot::InitTest() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  // configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif