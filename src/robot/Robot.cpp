#include "Robot.h"

#include "RobotConfig.h"

void Robot::begin() {
    Serial.begin(115200);
    pinMode(cfg::kStartButtonPin, INPUT_PULLUP);
    pinMode(cfg::kStatusLedPin, OUTPUT);
    digitalWrite(cfg::kStatusLedPin, LOW);

    motors_.begin();
    sensors_.begin();
    state_ = sensors_.calibrationValid() ? RobotState::WAIT_START
                                            : RobotState::CALIBRATION_REQUIRED;
    stateStartedMs_ = millis();
    lastControlUs_ = micros();

    Serial.println();
    Serial.println(F("Linea Viper X16 competition controller / ESP32-S2"));
    if (sensors_.calibrationValid()) {
        Serial.println(F("Calibration loaded. Tap START to arm, hold START to calibrate."));
    } else {
        Serial.println(F("No valid calibration. Hold START for one second to calibrate."));
    }
    Serial.println(F("Console: r=run, c=calibrate, x=stop, s=status, t=telemetry"));
}

void Robot::update() {
    pollConsole();
    pollButton();
    motors_.update(micros());

    switch (state_) {
        case RobotState::CALIBRATING:
            updateCalibration();
        break;
        case RobotState::RUNNING:
            updateRunning();
        break;
        case RobotState::JUNCTION_ENTRY:
        case RobotState::JUNCTION_TURN:
            updateJunction();
        break;
        case RobotState::LINE_RECOVERY:
            updateRecovery();
        break;
        case RobotState::CALIBRATION_REQUIRED:
        case RobotState::WAIT_START:
        case RobotState::FINISHED:
        case RobotState::FAULT:
        break;
    }

    // A short heartbeat without slowing the control loop.
    const bool on = state_ == RobotState::RUNNING || state_ == RobotState::JUNCTION_ENTRY ||
                    state_ == RobotState::JUNCTION_TURN;
    const bool blink = ((millis() / 250U) & 1U) != 0U;
    digitalWrite(cfg::kStatusLedPin, on ? HIGH : (blink && state_ != RobotState::WAIT_START));
}

void Robot::pollButton() {
    const bool down = digitalRead(cfg::kStartButtonPin) == LOW;
    const uint32_t now = millis();

    if (down && !buttonWasDown_) {
        buttonPressedMs_ = now;
        longPressHandled_ = false;
    }

    const bool canCalibrate = state_ == RobotState::WAIT_START ||
                                state_ == RobotState::CALIBRATION_REQUIRED ||
                                state_ == RobotState::FINISHED || state_ == RobotState::FAULT;
    
    if (down && canCalibrate && !longPressHandled_ &&
        now - buttonPressedMs_ >= cfg::kLongPressMs) {
        longPressHandled_ = true;

        startCalibration();
    }

    if (!down && buttonWasDown_ && !longPressHandled_) {
        if (state_ == RobotState::WAIT_START && sensors_.calibrationValid()) {
            beginRun();
        } else if (state_ == RobotState::CALIBRATION_REQUIRED) {
            Serial.println(F("Calibration is required; hold START for one second."));
        }
    }

    buttonWasDown_ = down;
}

void Robot::pollConsole() {
    while (Serial.available() > 0) {
        const char command = static_cast<char>(Serial.read());
        switch (command) {
        case 'r':
        case 'R':
            if (sensors_.calibrationValid()) {
            beginRun();
            } else {
            Serial.println(F("Cannot run: Viper calibration is invalid."));
            }
            break;
        case 'c':
        case 'C':
            startCalibration();
            break;
        case 'x':
        case 'X':
            motors_.emergencyStop();
            controller_.reset();
            setState(sensors_.calibrationValid() ? RobotState::WAIT_START
                                                : RobotState::CALIBRATION_REQUIRED);
            Serial.println(F("Stopped. Tap START or send r to run."));
            break;
        case 's':
        case 'S':
            showStatus();
            break;
        case 't':
        case 'T':
            telemetryEnabled_ = !telemetryEnabled_;
            Serial.println(telemetryEnabled_ ? F("Telemetry enabled") : F("Telemetry disabled"));
            break;
        default:
            break;  // ignore line endings and unknown serial noise
        }
    }
}

void Robot::startCalibration() {
    motors_.emergencyStop();
    sensors_.startCalibration();
    controller_.reset();
    junctionLocked_ = false;
    setState(RobotState::CALIBRATING);
    Serial.println(F("Calibrating for 4.8 s: robot will sweep left/right. Keep it over line and floor."));
}

void Robot::updateCalibration() {
    const uint32_t elapsed = millis() - stateStartedMs_;
    const int8_t direction = ((elapsed / cfg::kCalibrationPhaseMs) & 1U) == 0U ? 1 : -1;

    motors_.command(-direction * cfg::kCalibrationTurnPwm,
                    direction * cfg::kCalibrationTurnPwm);
    sensors_.sampleCalibration();

    if (elapsed < cfg::kCalibrationDurationMs) {
        return;
    }

    motors_.emergencyStop();

    if (sensors_.finishCalibration()) {
        setState(RobotState::WAIT_START);
        Serial.println(F("Calibration saved. Tap START to run."));
    } else {
        setState(RobotState::CALIBRATION_REQUIRED);
        Serial.println(F("Calibration failed: every X16 channel must see both line and floor. Try again."));
    }
}

void Robot::beginRun() {
    if (!sensors_.calibrationValid()) {
        return;
    }

    controller_.reset();

    lostSinceMs_ = 0;
    stopSinceMs_ = 0;

    lastSeenPosition_ = 0;
    junctionLocked_ = false;
    lastControlUs_ = micros();

    setState(RobotState::RUNNING);
    Serial.println(F("RUN"));
}

void Robot::updateRunning() {
    sensors_.read(lastFrame_);
    const uint32_t now = millis();

    if (lastFrame_.lineVisible) {
        lastSeenPosition_ = lastFrame_.position;
        lostSinceMs_ = 0;
    } else {
        if (lostSinceMs_ == 0) {
            lostSinceMs_ = now;
        }

        if (now - lostSinceMs_ >= cfg::kLineLostConfirmMs) {
            startRecovery();
            return;
        }
    }

    if (cfg::kEnableStopLine && lastFrame_.activeCount >= cfg::kStopActiveSensors &&
        lastFrame_.strength >= cfg::kStopStrength) {
        if (stopSinceMs_ == 0) {
            stopSinceMs_ = now;
        }

        if (now - stopSinceMs_ >= cfg::kStopHoldMs) {
            finish();
            return;
        }
    } else {
        stopSinceMs_ = 0;
    }

    if (!junctionPresent(lastFrame_)) {
        junctionLocked_ = false;
    } else if (!junctionLocked_) {
        junctionLocked_ = true;
        pendingTurn_ = nextRouteAction();
        controller_.reset();

        setState(RobotState::JUNCTION_ENTRY);
        Serial.print(F("Junction: "));
        if (pendingTurn_ == JunctionAction::LEFT) Serial.println(F("left"));
        if (pendingTurn_ == JunctionAction::RIGHT) Serial.println(F("right"));
        if (pendingTurn_ == JunctionAction::STRAIGHT) Serial.println(F("straight"));

        return;
    }

    const uint32_t nowUs = micros();
    const float dt = (nowUs - lastControlUs_) / 1000000.0F;
    lastControlUs_ = nowUs;
    lastControl_ = controller_.update(lastFrame_.position, dt);
    motors_.command(lastControl_.left, lastControl_.right);
    emitTelemetry();
}

void Robot::updateJunction() {
    sensors_.read(lastFrame_);
    const uint32_t elapsed = millis() - stateStartedMs_;
    if (state_ == RobotState::JUNCTION_ENTRY) {
        motors_.command(cfg::kJunctionEntryPwm, cfg::kJunctionEntryPwm);
        if (elapsed >= cfg::kJunctionEntryMs) {
            setState(RobotState::JUNCTION_TURN);
        }

        return;
    }

    if (pendingTurn_ == JunctionAction::STRAIGHT) {
        motors_.command(cfg::kJunctionEntryPwm, cfg::kJunctionEntryPwm);
        if (elapsed >= cfg::kStraightTraverseMs) {
            setState(RobotState::RUNNING);
            lastControlUs_ = micros();
        }

        return;
    }

    const bool left = pendingTurn_ == JunctionAction::LEFT;
    motors_.command(left ? cfg::kTurnInsidePwm : cfg::kTurnOutsidePwm,
                    left ? cfg::kTurnOutsidePwm : cfg::kTurnInsidePwm);
    if (elapsed >= cfg::kMinimumTurnMs && centreReacquired(lastFrame_)) {
        controller_.reset();
        setState(RobotState::RUNNING);
        lastControlUs_ = micros();
    } else if (elapsed >= cfg::kMaximumTurnMs) {
        Serial.println(F("Turn did not reacquire a line."));
        startRecovery();
    }
}

void Robot::startRecovery() {
    controller_.reset();
    setState(RobotState::LINE_RECOVERY);
    Serial.println(F("Line lost: searching last-known side."));
}

void Robot::updateRecovery() {
    sensors_.read(lastFrame_);
    const uint32_t elapsed = millis() - stateStartedMs_;
    if (lastFrame_.lineVisible) {
        controller_.reset();
        lastControlUs_ = micros();
        setState(RobotState::RUNNING);
        Serial.println(F("Line reacquired."));
        return;
    }
    if (elapsed >= cfg::kRecoveryTimeoutMs) {
        fault(F("line not found before recovery timeout"));
        return;
    }

    // If the last centroid was to the right, a positive left-wheel command and
    // negative right-wheel command yaw the chassis right about its axle centre.
    const int16_t direction = lastSeenPosition_ >= 0 ? 1 : -1;
    motors_.command(direction * cfg::kRecoveryTurnPwm,
                    -direction * cfg::kRecoveryTurnPwm);
}

void Robot::finish() {
    motors_.emergencyStop();
    setState(RobotState::FINISHED);
    Serial.println(F("Stop marker confirmed. Robot finished."));
}

void Robot::fault(const __FlashStringHelper* reason) {
    motors_.emergencyStop();
    setState(RobotState::FAULT);
    Serial.print(F("FAULT: "));
    Serial.println(reason);
}

void Robot::setState(RobotState state) {
    state_ = state;
    stateStartedMs_ = millis();
}

bool Robot::centreReacquired(const SensorFrame& frame) const {
    return frame.lineVisible && abs(frame.position) <= cfg::kTurnReacquirePosition &&
            (frame.line[7] >= cfg::kActiveThreshold || frame.line[8] >= cfg::kActiveThreshold);
}

bool Robot::junctionPresent(const SensorFrame& frame) const {
    return frame.wide && (frame.leftEdge || frame.rightEdge);
}

JunctionAction Robot::nextRouteAction() {
    const JunctionAction action = cfg::kRoute[routeIndex_ % cfg::kRouteLength];
    ++routeIndex_;
    return action;
}

void Robot::showStatus() const {
    Serial.print(F("state="));
    Serial.print(stateName());
    Serial.print(F(" calibrated="));
    Serial.print(sensors_.calibrationValid() ? F("yes") : F("no"));
    Serial.print(F(" position="));
    Serial.print(lastFrame_.position);
    Serial.print(F(" strength="));
    Serial.print(lastFrame_.strength);
    Serial.print(F(" active="));
    Serial.println(lastFrame_.activeCount);
}

void Robot::emitTelemetry() {
    if (!telemetryEnabled_ || millis() - lastTelemetryMs_ < cfg::kTelemetryIntervalMs) {
        return;
    }
    
    lastTelemetryMs_ = millis();
    Serial.print(F("p="));
    Serial.print(lastFrame_.position);
    Serial.print(F(" a="));
    Serial.print(lastFrame_.activeCount);
    Serial.print(F(" q="));
    Serial.print(lastFrame_.strength);
    Serial.print(F(" e="));
    Serial.print(lastControl_.error, 3);
    Serial.print(F(" l="));
    Serial.print(lastControl_.left);
    Serial.print(F(" r="));
    Serial.println(lastControl_.right);
}

const __FlashStringHelper* Robot::stateName() const {
    switch (state_) {
        case RobotState::CALIBRATION_REQUIRED: return F("CALIBRATION_REQUIRED");
        case RobotState::WAIT_START: return F("WAIT_START");
        case RobotState::CALIBRATING: return F("CALIBRATING");
        case RobotState::RUNNING: return F("RUNNING");
        case RobotState::JUNCTION_ENTRY: return F("JUNCTION_ENTRY");
        case RobotState::JUNCTION_TURN: return F("JUNCTION_TURN");
        case RobotState::LINE_RECOVERY: return F("LINE_RECOVERY");
        case RobotState::FINISHED: return F("FINISHED");
        case RobotState::FAULT: return F("FAULT");
    }

    return F("UNKNOWN");
}
