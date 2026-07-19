#include <Arduino.h>

#include "main.h"
#include "decisions.h"
#include "motors.h"
#include "calculations.h"

void movement() {
    streamSensors();
    
    if (activeBlackCount >= 7 && totalSum > 7400) { avoid(); return; }
    if (activeGrayCount >= 5 && activeBlackCount <= 2) { terminate(); return; }
    if (normalized[0] > GRID_THRESHOLD && normalized[NUM_SENSORS - 1] > GRID_THRESHOLD) { blindForward(35); return; }

    pathDetect();
    
    int position = weightedPos(normalized, activeBlackCount);
    PID(position);

    turnBias = 0;
    
    delayMicroseconds(500);
}