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
  /*
    definitely insert stuff here
    What do you need to set the target to?
  */
}

void IntakeCommand::OnEnd(bool interrupted) {
  /* is there stuff here? */ 
}

bool IntakeCommand::IsFinished() {
  /* insert stuff here */
  /* When should this command end... perhaps when you are at the position? deadband?*/
  return false;
}