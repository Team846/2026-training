#include "subsystems/hardware/intake.h"

#include "frc846/control/calculators/CircuitResistanceCalculator.h"

IntakeSubsystem::IntakeSubsystem()
    : GenericSubsystem("intake"),
      motor_configs_{.can_id = 21,
          .inverted = false,
          .brake_mode = true,
          .motor_current_limit = 40_A,
          .smart_current_limit = 30_A,
          .voltage_compensation = 12_V,
          .circuit_resistance = frc846::wpilib::unit_ohm{0.01},
          .rotational_inertia = frc846::wpilib::unit_kg_m_sq{1.0}},
      esc_{frc846::control::base::SPARK_MAX_VORTEX, motor_configs_} {
  esc_helper_.bind(&esc_);
  esc_helper_.SetConversion(1_tr / 1_tr);
  esc_helper_.SetSoftLimits({false, 360_deg, -360_deg, 340_deg, -340_deg, 0.2});

  // What target position do you want it to go to?
  // Look at the challenge...
  RegisterPreference("k_p", 0.0);
  RegisterPreference("k_d", 0.0);
}

IntakeTarget IntakeSubsystem::ZeroTarget() const { return {0.0_deg}; }

void IntakeSubsystem::Setup() {
  // DO NOT CHANGE ANYTHING HERE
  esc_.Setup();

  esc_.EnableStatusFrames({
      frc846::control::config::StatusFrame::kPositionFrame,
      frc846::control::config::StatusFrame::kVelocityFrame,
      frc846::control::config::StatusFrame::kFaultFrame,
  });

  esc_helper_.SetPosition(0_deg);
}

bool IntakeSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_.VerifyConnected(), ok, "Could not verify intake motor");
  return ok;
}

IntakeReadings IntakeSubsystem::ReadFromHardware() {
  Graph("readings/position", position of the intake);
  // What readings do you need for PID? only do P and D
  // position is one... whats the other one?
  return {};
}

void IntakeSubsystem::WriteToHardware(IntakeTarget target) {
  IntakeReadings readings = GetReadings();
  // change this to use PID using the readings and the target
  // Remember PID = k_P * (position_error) - k_D * (velocity)
  // Remember to tune the value of k_P you can use use the prefrence...  GetPreferenceValue_double("k_p")
  double duty_cycle = pid here;

  Graph("target/dc", duty_cycle);
  esc_helper_.WriteDC(duty_cycle);
}