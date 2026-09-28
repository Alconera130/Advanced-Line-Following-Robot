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
#if defined(ROBOT_TARGET_C3_SUPERMINI)
    /*
        ESP32-C3 SuperMini temporary wiring. STBY is tied directly to 3.3 V;
        GPIO9 is the board's BOOT button and becomes the start/calibrate button
        after boot. GPIO2 is a boot strap pin: the Viper S0 input must not pull
        it low during reset (a 10 kOhm pull-up to 3.3 V is recommended).
    */
    constexpr uint8_t kViperAdcPin = 1;  // ADC1_CH1, Viper mux SIG/OUT
    constexpr uint8_t kMuxSelectPins[4] = {2, 3, 4, 5};  // Viper S0..S3
    constexpr uint8_t kStartButtonPin = 9;  // on-board BOOT button to GND
    constexpr int8_t kStatusLedPin = -1;  // GPIO8 LED left unused on this map

    constexpr uint8_t kLeftIn1Pin = 6;   // TB6612 AIN1
    constexpr uint8_t kLeftIn2Pin = 7;   // TB6612 AIN2
    constexpr uint8_t kLeftPwmPin = 10;  // TB6612 PWMA
    constexpr uint8_t kRightIn1Pin = 20; // TB6612 BIN1
    constexpr uint8_t kRightIn2Pin = 21; // TB6612 BIN2
    constexpr uint8_t kRightPwmPin = 0;  // TB6612 PWMB
    constexpr int8_t kMotorStandbyPin = -1;  // wire TB6612 STBY to 3.3 V
#else
    constexpr uint8_t kViperAdcPin = 1;  // ADC1_CH0, Viper mux SIG/OUT
    // Requested wiring retained where there is no conflict: S1=GPIO39 and S3=GPIO38.
    constexpr uint8_t kMuxSelectPins[4] = {2, 37, 4, 39};  // Viper S0, S1, S2, S3
    constexpr uint8_t kStartButtonPin = 6;                // button to GND
    constexpr int8_t kStatusLedPin = LED_BUILTIN;

    // TB6612FNG-style motor interface. GPIO38 cannot also be AIN1 because it is
    // already the Viper S3 signal; GPIO37 is the conflict-free replacement.
    constexpr uint8_t kLeftIn1Pin = 38;   // AIN1
    constexpr uint8_t kLeftIn2Pin = 34;   // AIN2
    constexpr uint8_t kLeftPwmPin = 8;    // PWMA
    constexpr uint8_t kRightIn1Pin = 13;  // BIN1
    constexpr uint8_t kRightIn2Pin = 10;   // BIN2
    constexpr uint8_t kRightPwmPin = 14;  // PWMB
    constexpr int8_t kMotorStandbyPin = -1;  // TB6612 STBY; set -1 for drivers without it
#endif
    constexpr uint8_t kLeftPwmChannel = 0;
    constexpr uint8_t kRightPwmChannel = 1;
    constexpr uint32_t kMotorPwmHz = 20000;
    constexpr uint8_t kMotorPwmResolution = 8;

    // The sensor output's electrical direction is fixed by the X16 hardware. Set
    // this false if telemetry shows a white surface reads lower than a black one.
    constexpr bool kSensorOutputIncreasesWithReflectance = true;
    // Starting preference while the automatic mode classifier settles after boot.
    constexpr bool kDefaultLineIsDark = true;
    constexpr bool kEnableAutomaticPolarity = true;
    constexpr uint8_t kPolaritySwitchConfirmFrames = 5;
    constexpr int16_t kPolaritySwitchScoreMargin = 90;
    constexpr uint8_t kNominalLineMaximumSensors = 6;
    constexpr uint8_t kUniformBackgroundSensors = 13;
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
    constexpr float kPidP = 75.0F;
    constexpr float kPidI = 10.0F;
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

    // Cross small breaks/dots by holding the final steering vector briefly. This
    // prevents a normal dotted-line gap from becoming a search manoeuvre.
    constexpr uint16_t kBrokenLineBridgeMs = 110;
    constexpr int16_t kGapBridgePwm = 118;
    constexpr float kGapCorrectionScale = 0.65F;
    constexpr uint16_t kRecoveryBeforeDeadEndMs = 360;
    constexpr uint16_t kRecoveryTimeoutMs = 900;
    constexpr int16_t kRecoveryTurnPwm = 115;

    // A long absence is treated as a likely dead end. The robot makes an open-loop
    // U-turn, re-acquires the return line, then uses its junction-path stack to
    // choose another visible branch. Tune these times on the real chassis.
    constexpr bool kEnableDeadEndReturn = true;
    constexpr uint16_t kDeadEndMinimumTurnMs = 330;
    constexpr uint16_t kDeadEndMaximumTurnMs = 1150;
    constexpr int16_t kDeadEndTurnPwm = 128;
    constexpr uint8_t kMaximumDeadEndReturns = 8;
    constexpr uint8_t kPathMemoryDepth = 16;

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
}
