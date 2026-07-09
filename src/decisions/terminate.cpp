#include <Arduino.h>

#include "main.h"
#include "decisions.h"
#include "motors.h"

void terminate() {
    setMotorSpeeds(0, 0);
    Serial.println("[GOAL LINE REACHED] Safe Shutdown.");
    while(1) { delay(1000); }
}