#include <Arduino.h>

#include "main.h"
#include "motors.h"

void calibrateSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }
    
    Serial.println("[ESP32-S3 SENSOR CALIBRATION STARTING]");
    unsigned long startTime = millis();
    
    while (millis() - startTime < 5000) {
        for (int i = 0; i < NUM_SENSORS; i++) {
        int reading = analogRead(SENSOR_PINS[i]);
        if (reading < sensorMin[i]) sensorMin[i] = reading;
        if (reading > sensorMax[i]) sensorMax[i] = reading;
        }
        delay(5);
    }

    Serial.println("[CALIBRATION SETTINGS LOADED]");
}