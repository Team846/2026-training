#include "subsystems/hardware/leds.h"

LEDsSubsystem::LEDsSubsystem()
    : frc846::robot::GenericSubsystem<LEDsReadings, LEDsTarget>("leds") {}

void LEDsSubsystem::Setup() {
  leds_.SetLength(kLength);
  leds_.SetData(leds_buffer_);
  leds_.Start();
}

LEDsTarget LEDsSubsystem::ZeroTarget() const {
  return {kDefault};
}

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::ReadFromHardware() {
  return {};
}

void LEDsSubsystem::WriteToHardware(LEDsTarget target) {
  // Example of how to set LEDs
  if (target.state==kRed){
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(255, 0, 0);
    }
  } else if (target.state==kBlue){
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(0, 0, 255);
    }
  } else if (target.state==kGreen){
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(0, 255, 0);
    }
  } else {
    for (int i=0; i<kLength; i++){
      leds_buffer_[i].SetRGB(0, 0, 0);
    }
  }

  // Use target.state to change the LEDs

  leds_.SetData(leds_buffer_);  // DO NOT REMOVE
}