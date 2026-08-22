#include "commands/leds_command.h"

LEDsCommand::LEDsCommand(RobotContainer &container)
    : frc846::robot::GenericCommand<RobotContainer, LEDsCommand>{
          container, "leds_command"} {
  AddRequirements({&container.leds_});//What is required by the subsystem?
}

void LEDsCommand::OnInit() {
  loops = 0;
  //Should you do anything here?
}

void LEDsCommand::Periodic() {
  //Blink the leds red, green and blue with each color lasting for .5 second (25 loops)
  LEDsTarget targ;
  if (loops % 75 < 25) {
    targ={kRed};
  } else if(loops % 75 < 50) {
    targ={kGreen};
  } else if (loops % 75 < 75) {
    targ={kBlue};
  } else {
    targ={kDefault};
  }
  loops++;
  container_.leds_.SetTarget(targ);
}

void LEDsCommand::OnEnd(bool interrupted) {
  //Go back to the green color
  container_.leds_.SetTarget({kGreen});
}

bool LEDsCommand::IsFinished() { 
    //finishes when each color has been seen three times
    return loops > 75*3;
 }