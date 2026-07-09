#include <Arduino.h>

#include "main.h"
#include "motors.h"

void PID(int position) {
    int error = position - 0;
  
    float pTerm = Kp * error;
    float dTerm = Kd * (error - lastError);
    integral   += error;
    float iTerm = Ki * integral;
    
    int steeringCorrection = (int)(pTerm + iTerm + dTerm);
    lastError = error;
    
    int leftMotorPWMValue  = baseSpeed + steeringCorrection;
    int rightMotorPWMValue = baseSpeed - steeringCorrection;
    
    setMotorSpeeds(leftMotorPWMValue, rightMotorPWMValue);
}