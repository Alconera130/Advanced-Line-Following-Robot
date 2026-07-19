#include <Arduino.h>

#include "main.h"
#include "decisions.h"
#include "motors.h"

void pathDetect(bool priority = false) {
    bool left = normalized[0] > GRID_THRESHOLD;
    bool right = normalized[NUM_SENSORS - 1] > GRID_THRESHOLD;
    bool center = normalized[NUM_SENSORS / 2] > GRID_THRESHOLD;

    if (priority) { 
        if (left && right) Serial.println("[PATH DETECTED] Both sides");
        else if (left) Serial.println("[PATH DETECTED] Left side"); 
        else if (right) Serial.println("[PATH DETECTED] Right side");
        else return;
    } else {
        if (left && right && center) {
            const char direction = steps[step++];
            
            switch (direction) {
                case 'L':
                    Serial.println("[PATH DETECTED] Both sides, turning left");
                    turnBias = -80;
                    delay(300);
                    break;
                case 'R':
                    Serial.println("[PATH DETECTED] Both sides, turning right");
                    turnBias = 80;
                    delay(300);
                    break;
                case 'S':
                    Serial.println("[PATH DETECTED] Both sides, moving straight");
                    turnBias = 0;
                    delay(300);
                    break;
            }
        }
    }
}