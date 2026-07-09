#pragma once

#define NUM_SENSORS 8

extern const int SENSOR_PINS[NUM_SENSORS];

extern const int ENA;
extern const int IN1;
extern const int IN2;

extern const int ENB;
extern const int IN3;
extern const int IN4;

extern const int pwmFreq;
extern const int pwmResolution;

extern const int leftPWMChannel;
extern const int rightPWMChannel;

extern const float Kp;
extern const float Ki;
extern const float Kd;

extern const int baseSpeed;
extern const int maxSpeed;

extern const int GRID_THRESHOLD;

extern int sensorMin[NUM_SENSORS];
extern int sensorMax[NUM_SENSORS];

extern int lastError;
extern float integral;

extern int rawValues[NUM_SENSORS];
extern int normalized[NUM_SENSORS];
extern long totalSum;
extern int activeBlackCount;
extern int activeGrayCount;