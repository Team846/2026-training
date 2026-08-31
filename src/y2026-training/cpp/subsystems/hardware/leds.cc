#include "subsystems/hardware/leds.h"

LEDsSubsystem::LEDsSubsystem()
    : frc846::robot::GenericSubsystem<LEDsReadings, LEDsTarget>("leds") {}

void LEDsSubsystem::Setup() {
  leds_.SetLength(kLength);
  leds_.SetData(leds_buffer_);
  leds_.Start();
}

LEDsTarget LEDsSubsystem::ZeroTarget() const {
  /* What is your default LED State?*/
  return {};
}

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::ReadFromHardware() {
  // Are there any readings?
  return {};
}

void LEDsSubsystem::WriteToHardware(LEDsTarget target) {
  // Example of how to set LEDs
  for (int i = 0; i < kLength; i++) {
    leds_buffer_[i].SetRGB(0, 0, 0);
  }

  // Use target.state to change the LEDs

  leds_.SetData(leds_buffer_);  // DO NOT REMOVE
}