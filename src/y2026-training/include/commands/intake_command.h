#pragma once

#include "frc846/robot/GenericCommand.h"
#include "subsystems/robot_container.h"
#include <units/angle.h>

class IntakeCommand
    : public frc846::robot::GenericCommand<RobotContainer, IntakeCommand> {
 public:
  IntakeCommand(RobotContainer& container, units::degree_t target_position);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

 private:
  units::degree_t target_position_;
};