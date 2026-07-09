#include <Arduino.h>

#include "main.h"
#include "decisions.h"
#include "motors.h"
#include "calculations.h"

void movement() {
    streamSensors();
    avoid();
    terminate();

    if (normalized[0] > GRID_THRESHOLD && normalized[NUM_SENSORS - 1] > GRID_THRESHOLD) {
        blindForward(35);
        return;
    }

    int position = weightedPos(normalized, activeBlackCount);
    PID(position);
    
    delayMicroseconds(500);
}