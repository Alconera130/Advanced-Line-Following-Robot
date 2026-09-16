#include <Arduino.h>

#include "main.h"
#include "motors.h"
#include "decisions.h"

void blindForward(int durationMs) {
    setMotorSpeeds(baseSpeed, baseSpeed);
    delay(durationMs); 
}