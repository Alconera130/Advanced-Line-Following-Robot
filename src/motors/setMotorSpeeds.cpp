#include <Arduino.h>
#include "main.h"
#include "motors.h"

int minSpeed = 40;
float accelRate = 0.2;

int currentLeft = 0;
int currentRight = 0;

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    currentLeft  = currentLeft  + (leftSpeed  - currentLeft)  * accelRate;
    currentRight = currentRight + (rightSpeed - currentRight) * accelRate;

    leftSpeed  = currentLeft;
    rightSpeed = currentRight;

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

    if (leftSpeed > 0 && leftSpeed < minSpeed) leftSpeed = minSpeed;
    if (rightSpeed > 0 && rightSpeed < minSpeed) rightSpeed = minSpeed;

    leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);

    ledcWrite(leftPWMChannel, leftSpeed);
    ledcWrite(rightPWMChannel, rightSpeed);
}