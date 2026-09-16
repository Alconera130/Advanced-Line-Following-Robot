#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

/*
    Pin map for a WEMOS LOLIN S2 Mini. All signals are 3.3 V logic.
    The Viper X16 pictured by the user appears to have its 16:1 analogue mux
    onboard. These are its select inputs and common SIG output, not an extra mux.
    Confirm the connector labels/IC marking before applying power.
*/

namespace cfg {
constexpr uint8_t kViperAdcPin = 1;  // ADC1_CH0, Viper mux SIG/OUT
constexpr uint8_t kMuxSelectPins[4] = {2, 3, 4, 5};  // Viper S0, S1, S2, S3
constexpr uint8_t kStartButtonPin = 12;              // button to GND
constexpr uint8_t kStatusLedPin = LED_BUILTIN;

constexpr uint8_t kLeftIn1Pin = 6;
constexpr uint8_t kLeftIn2Pin = 7;
constexpr uint8_t kLeftPwmPin = 8;
constexpr uint8_t kRightIn1Pin = 9;
constexpr uint8_t kRightIn2Pin = 10;
constexpr uint8_t kRightPwmPin = 11;
constexpr uint8_t kLeftPwmChannel = 0;
constexpr uint8_t kRightPwmChannel = 1;
constexpr uint32_t kMotorPwmHz = 20000;
constexpr uint8_t kMotorPwmResolution = 8;

// Set false only if the line is more reflective than its background. The
// controller always sees a strong line as 1000, regardless of sensor polarity.
constexpr bool kLineIsDark = true;
constexpr uint8_t kSamplesPerSensor = 2;
constexpr uint16_t kMuxSettleUs = 4;
constexpr uint16_t kMinimumCalibrationSpan = 120;
constexpr float kSensorFilterAlpha = 0.48F;
constexpr uint16_t kActiveThreshold = 280;
constexpr uint16_t kLineVisibleStrength = 75;
constexpr uint16_t kBranchThreshold = 620;
constexpr uint8_t kJunctionActiveSensors = 7;

// PID uses normalised lateral error (-1.0 to +1.0), then produces PWM units.
// Tune kPidP first, then kPidD; use a little kPidI only for a persistent bias.
constexpr float kPidP = 128.0F;
constexpr float kPidI = 12.0F;
constexpr float kPidD = 5.5F;
constexpr float kDerivativeFilter = 0.72F;
constexpr float kIntegralLimit = 0.55F;
constexpr int16_t kMaxCorrection = 175;
constexpr int16_t kCruisePwm = 195;
constexpr int16_t kCornerPwm = 92;
constexpr float kDerivativeSlowdown = 0.15F;

constexpr int16_t kMotorMaxPwm = 255;
constexpr int16_t kMotorMinimumPwm = 48;  // measure the real motor deadband
constexpr float kMotorSlewPwmPerSecond = 1200.0F;

// Route is consumed at each detected junction. Edit it to match a course.
constexpr JunctionAction kRoute[] = {
    JunctionAction::LEFT, JunctionAction::RIGHT, JunctionAction::STRAIGHT
};

constexpr uint8_t kRouteLength = sizeof(kRoute) / sizeof(kRoute[0]);
constexpr uint16_t kJunctionEntryMs = 38;
constexpr uint16_t kStraightTraverseMs = 105;
constexpr uint16_t kMinimumTurnMs = 75;
constexpr uint16_t kMaximumTurnMs = 700;
constexpr int16_t kJunctionEntryPwm = 112;
constexpr int16_t kTurnOutsidePwm = 150;
constexpr int16_t kTurnInsidePwm = -64;
constexpr int16_t kTurnReacquirePosition = 2500;

constexpr uint16_t kLineLostConfirmMs = 18;
constexpr uint16_t kRecoveryTimeoutMs = 700;
constexpr int16_t kRecoveryTurnPwm = 115;

// Disabled by default: course marking conventions differ between competitions.
constexpr bool kEnableStopLine = false;
constexpr uint8_t kStopActiveSensors = 13;
constexpr uint16_t kStopStrength = 760;
constexpr uint16_t kStopHoldMs = 180;

constexpr uint16_t kCalibrationDurationMs = 4800;
constexpr uint16_t kCalibrationPhaseMs = 600;
constexpr int16_t kCalibrationTurnPwm = 66;
constexpr uint16_t kLongPressMs = 1000;
constexpr uint16_t kTelemetryIntervalMs = 100;
}  // namespace cfg
