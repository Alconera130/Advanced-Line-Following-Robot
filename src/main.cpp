#include <Arduino.h>

#include "main.h"

const int SENSOR_PINS[NUM_SENSORS] = {2, 3, 4, 5, 6, 7, 8, 9};

const int ENA = 10;
const int IN1 = 11;
const int IN2 = 12;

const int ENB = 13;
const int IN3 = 14;
const int IN4 = 15;

const int pwmFreq = 1000;
const int pwmResolution = 8;

const int leftPWMChannel = 0;
const int rightPWMChannel = 1;

const float Kp = 0.45;
const float Ki = 0.0;
const float Kd = 3.80;

const int baseSpeed = 150;
const int maxSpeed = 255;

const int GRID_THRESHOLD = 3400;

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

int lastError;
float integral;