#include <Arduino.h>

#include "main.h"
#include "motors.h"

void PID(int position) {
    int error = position - 0;
  
    float pTerm = Kp * error;
    integral += error;

    if (integral > 10000) integral = 10000;
    if (integral < -10000) integral = -10000;

    float iTerm = Ki * integral;

    float rawDerivative = error - lastError;
    filteredDerivative = 0.7 * filteredDerivative + 0.3 * rawDerivative;
    float dTerm = Kd * filteredDerivative;

    int correction = (int)(pTerm + iTerm + dTerm);

    lastError = error;
    
    int speed = baseSpeed;
    int absError = abs(error);

    if (absError > 2000) {
        speed = baseSpeed - 60; // sharp turn
    } else if (absError > 1000) {
        speed = baseSpeed - 30; // medium turn
    }

    switch (turnState) {
        case LEFT:
            correction += 80; // bias to the left
            break;
        case RIGHT:
            correction -= 80; // bias to the right
            break;
        case NONE:
            break;  
    }

    if (turnState != NONE && abs(error) < 300) {
        turnState = NONE;
    }

    int leftMotorPWMValue  = speed + correction;
    int rightMotorPWMValue = speed - correction;

    leftMotorPWMValue  = constrain(leftMotorPWMValue, 0, maxSpeed);
    rightMotorPWMValue = constrain(rightMotorPWMValue, 0, maxSpeed);

    setMotorSpeeds(leftMotorPWMValue, rightMotorPWMValue);
}