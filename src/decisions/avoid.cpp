#include <Arduino.h>

#include "main.h"
#include "decisions.h"
#include "motors.h"

void avoid() {
    setMotorSpeeds(0, 0);       delay(100);
    setMotorSpeeds(170, -170);  delay(150); 
    setMotorSpeeds(210, 110);   delay(600); 
    setMotorSpeeds(90, 210);    delay(400); 
    setMotorSpeeds(120, 120);

    while(analogRead(SENSOR_PINS[3]) < GRID_THRESHOLD && analogRead(SENSOR_PINS[4]) < GRID_THRESHOLD) {
        delay(1); 
    }
}