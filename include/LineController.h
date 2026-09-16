#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class LineController {
    public:
        void reset();
        ControlOutput update(int16_t position, float dtSeconds);

    private:
        float previousError_ = 0.0F;
        float integral_ = 0.0F;
        float filteredDerivative_ = 0.0F;
};
