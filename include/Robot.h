#pragma once

#include <Arduino.h>

#include "LineController.h"
#include "MotorDriver.h"
#include "RobotTypes.h"
#include "ViperX16.h"

class Robot {
    public:
        void begin();
        void update();

    private:
        void pollButton();
        void pollConsole();
        void startCalibration();
        void updateCalibration();
        void beginRun();
        void updateRunning();
        void updateJunction();
        void startRecovery();
        void updateRecovery();
        void finish();
        void fault(const __FlashStringHelper* reason);
        void setState(RobotState state);
        void showStatus() const;
        void emitTelemetry();
        bool centreReacquired(const SensorFrame& frame) const;
        bool junctionPresent(const SensorFrame& frame) const;

        JunctionAction nextRouteAction();
        const __FlashStringHelper* stateName() const;

        ViperX16 sensors_;
        MotorDriver motors_;
        LineController controller_;

        RobotState state_ = RobotState::CALIBRATION_REQUIRED;
        JunctionAction pendingTurn_ = JunctionAction::STRAIGHT;

        SensorFrame lastFrame_{};
        ControlOutput lastControl_{};

        uint32_t stateStartedMs_ = 0;
        uint32_t lostSinceMs_ = 0;
        uint32_t stopSinceMs_ = 0;
        uint32_t lastTelemetryMs_ = 0;
        uint32_t lastControlUs_ = 0;
        uint32_t buttonPressedMs_ = 0;
        uint8_t routeIndex_ = 0;
        int16_t lastSeenPosition_ = 0;
        
        bool buttonWasDown_ = false;
        bool longPressHandled_ = false;
        bool telemetryEnabled_ = false;
        bool junctionLocked_ = false;
};
