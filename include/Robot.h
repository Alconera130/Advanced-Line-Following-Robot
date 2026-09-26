#pragma once

#include <Arduino.h>

#include "LineController.h"
#include "MotorDriver.h"
#include "RobotConfig.h"
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
        void bridgeBrokenLine();
        void startDeadEndTurn();
        void updateDeadEndTurn();
        void finish();
        void fault(const __FlashStringHelper* reason);
        void setState(RobotState state);
        void showStatus() const;
        void emitTelemetry();
        bool centreReacquired(const SensorFrame& frame) const;
        bool junctionPresent(const SensorFrame& frame) const;

        JunctionAction selectJunctionAction(const SensorFrame& frame);
        JunctionAction nextRouteAction(const SensorFrame& frame);
        JunctionAction firstAvailableAction(const SensorFrame& frame) const;
        bool actionAvailable(JunctionAction action, const SensorFrame& frame) const;
        void rememberJunctionAction(JunctionAction action);
        const __FlashStringHelper* stateName() const;

        struct PathMemoryEntry {
            JunctionAction action;
        };

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
        uint8_t pathDepth_ = 0;
        uint8_t deadEndReturns_ = 0;
        int16_t lastSeenPosition_ = 0;
        int8_t deadEndTurnDirection_ = 1;
        PathMemoryEntry pathMemory_[cfg::kPathMemoryDepth]{};
        
        bool buttonWasDown_ = false;
        bool longPressHandled_ = false;
        bool telemetryEnabled_ = false;
        bool junctionLocked_ = false;
        bool returningFromDeadEnd_ = false;
};
