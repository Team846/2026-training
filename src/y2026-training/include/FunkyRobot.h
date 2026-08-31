#pragma once

#include <frc/DigitalInput.h>

#include "frc846/robot/GenericRobot.h"
#include "subsystems/robot_container.h"
#include "commands/leds_command.h"

class FunkyRobot : public frc846::robot::GenericRobot {
public:
  FunkyRobot();

  void OnInitialize() override;

  void OnDisable() override;

  void OnPeriodic() override;

  void InitTeleop() override;
  void InitTest() override;

private:
  int teleopLoops = 0;
  RobotContainer container_;
};
