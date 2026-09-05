## Table of Contents

- [Table of Contents](#table-of-contents)
- [Intake Subsystem .h File](#intake-subsystem-h-file)
- [Intake Subsytem .cc File](#intake-subsytem-cc-file)
  - [Constructor](#constructor)
  - [Setup](#setup)
  - [ReadFromHardware](#readfromhardware)
  - [WriteToHardware](#writetohardware)
- [Superstructure .h file](#superstructure-h-file)
- [Superstructure .cc file](#superstructure-cc-file)
- [Registration in Robot Container](#registration-in-robot-container)
- [Registration in Robot Container](#registration-in-robot-container-1)
- [Teleop Command](#teleop-command)

## Intake Subsystem .h File

```cpp
enum class IntakeState { kIdle, kIntake, kEvac };
// kIdle: Low-speed idle spin.
// kIntake: High-speed forward intake.
// kEvac: Reverse outtake to eject jammed or unwanted game pieces.

// immutable struct for sensor readings
struct IntakeReadings {
  fps_t vel_;
};

// immutable struct for actuator demand
struct IntakeTarget {
  IntakeState target_state;
  fps_t dt_vel_; 
};

// subsystem implements read and write
class IntakeSubsystem
    : public funkit::robot::GenericSubsystem<IntakeReadings, IntakeTarget> {
public:
  IntakeSubsystem();
  ~IntakeSubsystem();

  void Setup() override;

  IntakeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  funkit::control::HigherMotorController esc_;

  fps_t trgt_vel_{0.0_fps_}; // initialize our target velocity. this can be changed.

  IntakeReadings ReadFromHardware() override;

  void WriteToHardware(IntakeTarget target) override;

  int reset_ctr_ = 0;
  int stall_ctr_ = 0;
};

```

## Intake Subsytem .cc File
### Constructor
```cpp

    IntakeSubsystem::IntakeSubsystem(): GenericSubsystem("Intake"), // name our subsystem "intake"
      esc_{base::TALON_FX_KRAKENX60, ports::intake_::kIntakeParams} { // register our motor and the CAN bus port

  // lets register our tunable preferences
  RegisterPreference("speed_idle", 15.0_fps_);
  RegisterPreference("speed_intake", 55.0_fps_);
  RegisterPreference("speed_evac", -25.0_fps_);
  RegisterPreference("dynamic_intake_gain", 1.0);
}
```

### Setup
```cpp
void IntakeSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 80_A_,
      .smart_current_limit = 80_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.001, .kI = 0.0, .kD = 0.0, .kF = 0.00127}}; // create a motor config object: current limit, brake mode, PIDF gains

  SubsystemGenomeHelper::CreateGenomePreferences(*this, "genome", genome_backup); // register our motor configs

  auto motor_specs = base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);
  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefLinearSys intake_plant(def_bldc, 1,
      20_rot_ / 12_rot_ * 1_rad_ / 2.0625_in_, 0.0_mps2_, 1.0_kg_, 0.5_N_,
      0.5_N_ / 700_radps_, 20_ms_);

  esc_.Setup(genome_backup, intake_plant);
  esc_.EnableStatusFrames({StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, 
                          20_ms_, 5_ms_, 5_ms_, 20_ms_);
  esc_.SetPosition(meter_t{0});
}
```

### ReadFromHardware
```cpp
IntakeReadings IntakeSubsystem::ReadFromHardware() {
  fps_t velocity_ = esc_.GetVelocity<mps_t>(); // get readings from our motor. 
  Graph("error", trgt_vel_ - velocity_); // graph the error (target velocity - current velocity)
  return IntakeReadings{velocity_}; // directly return our readings
}
```

### WriteToHardware
```cpp
void IntakeSubsystem::WriteToHardware(IntakeTarget target) {
  auto genome = SubsystemGenomeHelper::LoadGenomePreferences(*this, "genome");
  esc_.ModifyGenome(genome);

  if (target.target_state == IntakeState::kIntake) {
    // Dynamic Speed: Base target intake speed + Drivetrain velocity * gain
    trgt_vel_ = GetPreferenceValue_unit_type<fps_t>("speed_intake") +
                target.dt_vel_ * GetPreferenceValue_double("dynamic_intake_gain");

    
    if (u_abs(esc_.GetVelocity<mps_t>()) < 10_fps_) {
      stall_ctr_++;
    } else {
      stall_ctr_ = 0;
    }

    if (reset_ctr_ > 10 && reset_ctr_ <= 20) {
      trgt_vel_ = GetPreferenceValue_unit_type<fps_t>("speed_evac"); // Reverse to clear jam: set target to (-) velocity
    } else if (reset_ctr_ > 0) {
      // Allow motor to spin back up
    } else {
      if (stall_ctr_ > 25) { reset_ctr_ = 20; } // 25 consecutive stalled loops triggers evac
    }

    if (reset_ctr_ > 0) reset_ctr_--;
  } else if (target.target_state == IntakeState::kEvac) {
    trgt_vel_ = GetPreferenceValue_unit_type<fps_t>("speed_evac");
  } else {
    trgt_vel_ = GetPreferenceValue_unit_type<fps_t>("speed_idle");
    // (Idle stall clearing logic...)
  }

  Graph("target_velocity", trgt_vel_);
  esc_.WriteVelocity(trgt_vel_); // write our target velocity to the esc motor
}
```

## Superstructure .h file
```cpp

class HoptakeSuperstructure : public GenericSubsystem<HoptakeSSReadings, HoptakeSSTarget> {
 public:
  IntakeSubsystem intake; // register our subsystem
  PivotSubsystem pivot;
};
```

## Superstructure .cc file
```cpp
void HoptakeSuperstructure::Setup() {
  if (GetPreferenceValue_bool("init_intake")) {
    intake.InitByParent(); // Registers child name without duplicating WPILib scheduler registration
    intake.Setup();
  }
  if (GetPreferenceValue_bool("init_pivot")) {
    pivot.InitByParent();
    pivot.Setup();
  }
}

HoptakeSSReadings HoptakeSuperstructure::ReadFromHardware() {
  intake.UpdateReadings(); // Propagates read down to intake
  pivot.UpdateReadings();
  return HoptakeSSReadings{};
}

void HoptakeSuperstructure::WriteToHardware(HoptakeSSTarget target) {
  IntakeTarget intake_trgt{IntakeState::kIdle, 0.0_fps_};
  PivotTarget pivot_trgt{PivotState::kStow};

  // State Machine mapping high-level HoptakeState to child targets:
  if (target.target_state == HoptakeState::kCollapsed) {
    pivot_trgt.target_state = PivotState::kCollapsed;
    intake_trgt.target_state = IntakeState::kEvac;
  } else if (target.target_state == HoptakeState::kEvac) {
    pivot_trgt.target_state = PivotState::kIntake;
    intake_trgt.target_state = IntakeState::kEvac;
  } else if (target.target_state == HoptakeState::kIntake) {
    pivot_trgt.target_state = PivotState::kIntake;     // Lower pivot
    intake_trgt.target_state = IntakeState::kIntake;   // Spin rollers forward
  } else {
    pivot_trgt.target_state = PivotState::kStow;       // Raise pivot up
    intake_trgt.target_state = IntakeState::kIdle;      // Spin at idle
  }

  intake_trgt.dt_vel_ = target.drivetrain_vel;

  intake.SetTarget(intake_trgt);
  pivot.SetTarget(pivot_trgt);

  intake.UpdateHardware(); // Propagates write down to intake
  pivot.UpdateHardware();
}
```

## Registration in Robot Container
```cpp
class RobotContainer : public funkit::robot::GenericRobotContainer {
 public:
  HoptakeSuperstructure hoptake_ss_{};

  RobotContainer() {
    RegisterPreference("init_hoptake_ss", true);
    bool hoptake_ss_init = GetPreferenceValue_bool("init_hoptake_ss");

    RegisterSubsystemGroupB({{&hoptake_ss_, hoptake_ss_init}});
  }
};
```

## Registration in Robot Container
```cpp
class RobotContainer : public funkit::robot::GenericRobotContainer {
 public:
  HoptakeSuperstructure hoptake_ss_{};

  RobotContainer() {
    RegisterPreference("init_hoptake_ss", true); // we DO want to initialize this subsystem on startup. Register as a preference to potentially change this on the dashboard if we want to
    bool hoptake_ss_init = GetPreferenceValue_bool("init_hoptake_ss");

    RegisterSubsystemGroupB({{&hoptake_ss_, hoptake_ss_init}}); // register our subsystem and if we want to initialize on startup
  }
};
```
**GenericRobotContainer.h**
```cpp
void RegisterSubsystemGroupB(
    std::initializer_list<std::pair<funkit::robot::SubsystemBase*, bool>>
        subsystems) {
for (auto& [subsystem, init] : subsystems) {
    if (init) {
    subsystem->Init();
    group_b_subsystems_.push_back(subsystem);
    all_subsystems_.push_back(subsystem);
    }
}
}
```

## Teleop Command
**Input Processing**


```
ci_readings_.intake = dr_readings.left_trigger; // Driver holds left trigger to intake
```

```cpp
HoptakeCommand::HoptakeCommand(RobotContainer &container)
    : GenericCommand{container, "hoptake_command"} {
  AddRequirements({&container_.hoptake_ss_});
}

void HoptakeCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};
  HoptakeSSTarget target;

  if (ci_readings_.intake) {
    target.target_state = HoptakeState::kIntake;
  } else if (ci_readings_.evac_storage) {
    target.target_state = HoptakeState::kEvac;
  } else {
    target.target_state = HoptakeState::kIdle;
  }

  target.drivetrain_vel = container_.drivetrain_.GetReadings().pose.velocity.magnitude();
  container_.hoptake_ss_.SetTarget(target);
}
```