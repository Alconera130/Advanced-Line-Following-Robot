#include <Arduino.h>

#include "main.h"
#include "decisions.h"
#include "motors.h"

void pathDetect(bool priority = false) {
    bool left = normalized[0] > GRID_THRESHOLD;
    bool right = normalized[NUM_SENSORS - 1] > GRID_THRESHOLD;

    bool center = false;
    int width = NUM_SENSORS % 2 == 0 ? 2 : 3;

    for (int i = 0; i < width; i++) {
        center |= normalized[NUM_SENSORS / 2 - width / 2 + i] > GRID_THRESHOLD;
    }

    if (left && right && center) {
        if (priority) {
            Serial.println("[PATH DETECTED] Both sides and center");
            turnState = NONE;
        } else {
            const char direction = steps[step++ % STEP_LENGTH];
        
            switch (direction) {
                case 'L':
                    Serial.println("[PATH DETECTED] Both sides, turning left");
                    turnState = LEFT;
                    break;
                case 'R':
                    Serial.println("[PATH DETECTED] Both sides, turning right");
                    turnState = RIGHT;
                    break;
                case 'S':
                    Serial.println("[PATH DETECTED] Both sides, moving straight");
                    turnState = NONE;
                    break;
            }
        }
    }
    else if (left) { Serial.println("[PATH DETECTED] Left side"); turnState = LEFT; }
    else if (right) { Serial.println("[PATH DETECTED] Right side"); turnState = RIGHT; }
}