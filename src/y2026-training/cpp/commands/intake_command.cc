#include "commands/intake_command.h"

IntakeCommand::IntakeCommand(RobotContainer &container, units::degree_t target_position)
    : frc846::robot::GenericCommand<RobotContainer, IntakeCommand>{
          container, "intake_command"} {
  AddRequirements({&}); // add a requirement to the commmand etc. &container.leds_ let auto complete help you
  target_position_ = target_position;
}

void IntakeCommand::OnInit() {
   /* is there stuff here? */
}

void IntakeCommand::Periodic() {
  /* what should you do here? what needs to be set?*/
}

void IntakeCommand::OnEnd(bool interrupted) {
}

bool IntakeCommand::IsFinished() {
  return false;
}