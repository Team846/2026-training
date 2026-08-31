#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/hardware/intake.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
public:
  IntakeSubsystem intake_{};

  RobotContainer() {
    RegisterSubsystemGroupA({{&intake_, true}});

  }
};
