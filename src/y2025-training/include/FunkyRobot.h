#pragma once

#include <frc/DigitalInput.h>

#include "frc846/robot/GenericRobot.h"
#include "subsystems/robot_container.h"
#include "frc846/robot/xbox.h"
#include "commands/intake_command.h"

class FunkyRobot : public frc846::robot::GenericRobot {
public:
  FunkyRobot();

  void OnInitialize() override;

  void OnDisable() override;

  void OnPeriodic() override;

  void InitTeleop() override;
  void InitTest() override;

private:
  frc::XboxController xbox_controller_{0};
  RobotContainer container_;
};
