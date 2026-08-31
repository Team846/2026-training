#include "commands/leds_command.h"

LEDsCommand::LEDsCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, LEDsCommand>{
          container, "leds_command"} {
  AddRequirements({&});//What is required by the subsystem?
}

void LEDsCommand::OnInit() {
  
  //Should you do anything here?
}

void LEDsCommand::Periodic() {
  container_.leds_.
  //Blink the leds red, green and blue with each color lasting for .5 second (25 loops)

}

void LEDsCommand::OnEnd(bool interrupted) {
  //Go back to the green color
}

bool LEDsCommand::IsFinished() { 
    //finishes when each color has been seen three times
 }