#include <Arduino.h>

#include "main.h"
#include "motors.h"

void streamSensors() {
    int rawValues[NUM_SENSORS];
    int normalized[NUM_SENSORS];
    long totalSum = 0;
    int activeBlackCount = 0;
    int activeGrayCount = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        rawValues[i] = analogRead(SENSOR_PINS[i]); // Read raw values
        
        normalized[i] = map(rawValues[i], sensorMin[i], sensorMax[i], 0, 1000);
        normalized[i] = constrain(normalized[i], 0, 1000);
        
        totalSum += normalized[i];
        
        if (normalized[i] > 800) {
            activeBlackCount++;
        } else if (normalized[i] > 350 && normalized[i] <= 800) {
            activeGrayCount++;
        }
    }
}