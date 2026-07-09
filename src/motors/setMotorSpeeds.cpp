#include <Arduino.h>

#include "main.h"
#include "motors.h"

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    
    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        rightSpeed = -rightSpeed;
    }
    
    leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);
    
    ledcWrite(leftPWMChannel, leftSpeed);
    ledcWrite(rightPWMChannel, rightSpeed);
}