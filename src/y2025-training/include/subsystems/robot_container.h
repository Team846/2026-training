#pragma once

#include "frc846/robot/GenericRobotContainer.h"
#include "subsystems/hardware/leds.h"

class RobotContainer : public frc846::robot::GenericRobotContainer {
public:
  LEDsSubsystem leds_{};

  RobotContainer() {
    RegisterSubsystemGroupA({{&leds_, true}});
  }
};
