#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class ViperX16 {
    public:
        void begin();
        void read(SensorFrame& frame);

        void startCalibration();
        void sampleCalibration();
        bool finishCalibration();
        bool loadCalibration();
        bool saveCalibration() const;
        bool calibrationValid() const { return calibrationValid_; }

    private:
        uint16_t readChannel(uint8_t channel) const;
        uint16_t normalise(uint8_t channel, uint16_t raw) const;
        bool rangesAreValid() const;

        uint16_t minimum_[kViperSensorCount]{};
        uint16_t maximum_[kViperSensorCount]{};
        float filtered_[kViperSensorCount]{};
        bool filterInitialised_ = false;
        bool calibrationValid_ = false;
};
