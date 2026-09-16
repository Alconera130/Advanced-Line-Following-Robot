#pragma once

#include <Arduino.h>

constexpr uint8_t kViperSensorCount = 16;
constexpr int16_t kPositionLimit = 7500;

enum class RobotState : uint8_t {
  CALIBRATION_REQUIRED,
  WAIT_START,
  CALIBRATING,
  RUNNING,
  JUNCTION_ENTRY,
  JUNCTION_TURN,
  LINE_RECOVERY,
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
};

struct ControlOutput {
    int16_t left;
    int16_t right;
    int16_t correction;
    int16_t cruise;
    float error;
};
