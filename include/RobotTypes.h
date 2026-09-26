#pragma once

#include <Arduino.h>

constexpr uint8_t kViperSensorCount = 16;
constexpr int16_t kPositionLimit = 7500;

// The line's optical polarity after the sensor output has been normalised.
// DARK means a dark line on a lighter floor; LIGHT means a bright line on a
// darker floor. The sensor driver can change this automatically at a boundary.
enum class LinePolarity : uint8_t { DARK, LIGHT };

enum class RobotState : uint8_t {
  CALIBRATION_REQUIRED,
  WAIT_START,
  CALIBRATING,
  RUNNING,
  JUNCTION_ENTRY,
  JUNCTION_TURN,
  LINE_RECOVERY,
  DEAD_END_TURN,
  FINISHED,
  FAULT
};

enum class JunctionAction : uint8_t { LEFT, RIGHT, STRAIGHT };

struct SensorFrame {
    uint16_t raw[kViperSensorCount];
    uint16_t line[kViperSensorCount];  // 0 = floor, 1000 = line
    int16_t position;                  // -7500 = left, +7500 = right
    uint16_t strength;                 // mean line response, 0..1000
    uint8_t activeCount;
    bool lineVisible;
    bool leftEdge;
    bool rightEdge;
    bool wide;
    LinePolarity polarity;
    bool polarityChanged;
};

struct ControlOutput {
    int16_t left;
    int16_t right;
    int16_t correction;
    int16_t cruise;
    float error;
};
