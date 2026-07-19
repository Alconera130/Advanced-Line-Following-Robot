#include <Arduino.h>

#include "main.h"
#include "decisions.h"
#include "motors.h"
#include "calculations.h"

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

const float Kp = 0.45;  // For testing
const float Ki = 0.0;   // For testing
const float Kd = 3.80;  // For testing

const int baseSpeed = 150;
const int maxSpeed = 255;

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

int lastError;
float integral = 0;
float lastError = 0;
float filteredDerivative = 0;

int rawValues[NUM_SENSORS];
int normalized[NUM_SENSORS];
long totalSum = 0;
int activeBlackCount = 0;
int activeGrayCount = 0;

int step = 0;
const char steps[STEP_LENGTH] = { 'L', 'R', 'S' };
int turnBias = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcSetup(leftPWMChannel, pwmFreq, pwmResolution);
    ledcSetup(rightPWMChannel, pwmFreq, pwmResolution);

    ledcAttachPin(ENA, leftPWMChannel);
    ledcAttachPin(ENB, rightPWMChannel);

    setMotorSpeeds(0, 0);

    calibrateSensors();
}

void loop() {
    movement();
}