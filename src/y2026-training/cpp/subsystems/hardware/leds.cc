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
  // Are there any readings/updates you need each loop?
}

void LEDsSubsystem::WriteToHardware(LEDsTarget target) {
  // Example of how to set LEDs
  for (int i = 0; i < kLength; i++) {
    leds_buffer_[i].SetRGB(0, 0, 0);
  }

  // Decide what to set the LEDS to based on target.state 

  leds_.SetData(leds_buffer_);  // DO NOT REMOVE

  //DO NOT REMOVE, NEEDED FOR SIMULATION

  std::vector<double> rgb;
  for (const auto& led : leds_buffer_){
    rgb.insert(rgb.end(), {double(led.r), double(led.g), double(led.b)});
  }
  frc::SmartDashboard::PutNumberArray(name() + "/leds_rgb", rgb);
}