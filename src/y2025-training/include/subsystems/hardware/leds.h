#pragma once

#include <frc/AddressableLED.h>

#include "frc846/robot/GenericSubsystem.h"

enum LEDsState {
  kExampleStateOne,
  kExampleStateTwo,
  // TODO: Fix
};

struct LEDsReadings {
  // Are there any?
  // TODO: Fix
};

struct LEDsTarget {
  LEDsState state;
};

/*
DO NOT EDIT ANYTHING BELOW THIS.
GO TO LEDS.CC TO EDIT THE IMPLEMENTATION.
*/
class LEDsSubsystem
    : public frc846::robot::GenericSubsystem<LEDsReadings, LEDsTarget> {
 public:
  LEDsSubsystem();

  void Setup() override;

  LEDsTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  static constexpr int kLength = 30;

  std::array<frc::AddressableLED::LEDData, kLength> leds_buffer_;

  frc::AddressableLED leds_{6};  // LEDS connected to PWM Port 6

  LEDsReadings ReadFromHardware() override;

  int loops = 0;
  int first_pixel_hue_ = 0;

  void WriteToHardware(LEDsTarget target) override;
};