#pragma once

#include <time.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/math.h>
#include <units/velocity.h>

#include "frc846/base/Loggable.h"
#include "frc846/control/HMCHelper.h"
#include "frc846/control/HigherMotorController.h"
#include "frc846/robot/GenericSubsystem.h"

struct IntakeReadings {
    // what readings do you need?
};

struct IntakeTarget {
  units::degree_t target_position_;
};

class IntakeSubsystem
    : public frc846::robot::GenericSubsystem<IntakeReadings, IntakeTarget> {
public:
  IntakeSubsystem();

  IntakeTarget ZeroTarget() const override;

  frc846::control::config::MotorConstructionParameters GetCurrentConfig(
      frc846::control::config::MotorConstructionParameters original_config);

  void Setup() override;

  bool VerifyHardware() override;

protected:
  frc846::control::config::MotorConstructionParameters motor_configs_;

  frc846::control::HigherMotorController esc_;
  frc846::control::HMCHelper<units::degree> esc_helper_;

  IntakeReadings ReadFromHardware() override;

  void WriteToHardware(IntakeTarget target) override;
};
