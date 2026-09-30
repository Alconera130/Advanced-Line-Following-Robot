#include "Robot.h"

#include "RobotConfig.h"

void Robot::begin() {
    Serial.begin(115200);
    pinMode(cfg::kStartButtonPin, INPUT_PULLUP);
    if (cfg::kStatusLedPin >= 0) {
        pinMode(cfg::kStatusLedPin, OUTPUT);
        digitalWrite(cfg::kStatusLedPin, LOW);
    }

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
        case RobotState::DEAD_END_TURN:
            updateDeadEndTurn();
        break;
        case RobotState::CALIBRATION_REQUIRED:
        case RobotState::WAIT_START:
        case RobotState::FINISHED:
        case RobotState::FAULT:
        break;
    }

    // A short heartbeat without slowing the control loop.
    const bool on = state_ == RobotState::RUNNING || state_ == RobotState::JUNCTION_ENTRY ||
                    state_ == RobotState::JUNCTION_TURN || state_ == RobotState::DEAD_END_TURN;
    const bool blink = ((millis() / 250U) & 1U) != 0U;
    if (cfg::kStatusLedPin >= 0) {
        digitalWrite(cfg::kStatusLedPin,
                     on ? HIGH : (blink && state_ != RobotState::WAIT_START));
    }
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
    routeIndex_ = 0;
    junctionLocked_ = false;
    pathDepth_ = 0;
    deadEndReturns_ = 0;
    returningFromDeadEnd_ = false;
    resetLineLock();
    lastControlUs_ = micros();

    setState(RobotState::RUNNING);
    Serial.println(F("RUN"));
}

void Robot::updateRunning() {
    sensors_.read(lastFrame_);
    const uint32_t now = millis();

    if (lastFrame_.polarityChanged) {
        Serial.print(F("Line polarity switched to "));
        Serial.println(lastFrame_.polarity == LinePolarity::DARK ? F("dark") : F("light"));
    }

    const LineSegment* trackedSegment = selectTrackedSegment(lastFrame_);
    if (trackedSegment != nullptr) {
        trackedPosition_ = cfg::kEnableLineSegmentLock
                               ? trackedSegment->position
                               : lastFrame_.position;
        lastSeenPosition_ = trackedPosition_;
        lostSinceMs_ = 0;
    } else {
        if (lostSinceMs_ == 0) {
            lostSinceMs_ = now;
        }

        if (now - lostSinceMs_ <= cfg::kBrokenLineBridgeMs) {
            bridgeBrokenLine();
            return;
        }
        startRecovery();
            return;
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

    // A separated side line may make the full array look wide, but it must not
    // consume a route action. A genuine branch is merged with the tracked line
    // into one wide component before it is treated as a junction.
    if (!junctionPresent(lastFrame_, *trackedSegment)) {
        junctionLocked_ = false;
    } else if (!junctionLocked_) {
        junctionLocked_ = true;
        pendingTurn_ = selectJunctionAction(lastFrame_);
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
    lastControl_ = controller_.update(trackedPosition_, dt);
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
            resetLineLock();
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
        resetLineLock();
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
    const LineSegment* recoveredSegment = selectTrackedSegment(lastFrame_, true);
    if (recoveredSegment != nullptr) {
        trackedPosition_ = cfg::kEnableLineSegmentLock
                               ? recoveredSegment->position
                               : lastFrame_.position;
        lastSeenPosition_ = trackedPosition_;
        controller_.reset();
        lastControlUs_ = micros();
        setState(RobotState::RUNNING);
        Serial.println(F("Line reacquired."));
        return;
    }
    if (cfg::kEnableDeadEndReturn && elapsed >= cfg::kRecoveryBeforeDeadEndMs) {
        startDeadEndTurn();
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

void Robot::bridgeBrokenLine() {
    const int16_t correction = static_cast<int16_t>(constrain(
        lastControl_.correction * cfg::kGapCorrectionScale,
        -static_cast<float>(cfg::kMaxCorrection), static_cast<float>(cfg::kMaxCorrection)));
    motors_.command(constrain(cfg::kGapBridgePwm + correction,
                              -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm),
                    constrain(cfg::kGapBridgePwm - correction,
                              -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm));
    // Do not feed the gap duration into D on the first real reading afterwards.
    lastControlUs_ = micros();
}

void Robot::startDeadEndTurn() {
    if (deadEndReturns_ >= cfg::kMaximumDeadEndReturns) {
        fault(F("dead-end return limit reached"));
        return;
    }
    ++deadEndReturns_;
    // Select a consistent direction from the last observed line side. The value
    // is held through the U-turn so electrical noise cannot reverse it halfway.
    deadEndTurnDirection_ = lastSeenPosition_ >= 0 ? 1 : -1;
    setState(RobotState::DEAD_END_TURN);
    Serial.println(F("Likely dead end: executing memorised return turn."));
}

void Robot::updateDeadEndTurn() {
    sensors_.read(lastFrame_);
    const uint32_t elapsed = millis() - stateStartedMs_;
    motors_.command(deadEndTurnDirection_ * cfg::kDeadEndTurnPwm,
                    -deadEndTurnDirection_ * cfg::kDeadEndTurnPwm);

    if (elapsed >= cfg::kDeadEndMinimumTurnMs && centreReacquired(lastFrame_)) {
        controller_.reset();
        returningFromDeadEnd_ = true;
        junctionLocked_ = false;
        resetLineLock();
        lastControlUs_ = micros();
        setState(RobotState::RUNNING);
        Serial.println(F("Return line acquired; seeking next untried branch."));
        return;
    }
    if (elapsed >= cfg::kDeadEndMaximumTurnMs) {
        fault(F("dead-end U-turn did not reacquire a return line"));
    }
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

const LineSegment* Robot::selectTrackedSegment(const SensorFrame& frame,
                                                bool allowDistantReacquire) {
    if (!frame.lineVisible || frame.segmentCount == 0) {
        return nullptr;
    }

    const LineSegment* candidate = nullptr;
    int16_t smallestDistance = INT16_MAX;
    const int16_t reference = lineLockInitialised_ ? trackedPosition_ : 0;

    for (uint8_t i = 0; i < frame.segmentCount; ++i) {
        const LineSegment& segment = frame.segments[i];
        const int16_t distance = abs(segment.position - reference);
        if (candidate == nullptr || distance < smallestDistance ||
            (distance == smallestDistance && segment.strength > candidate->strength)) {
            candidate = &segment;
            smallestDistance = distance;
        }
    }

    if (candidate == nullptr) {
        return nullptr;
    }

    // During normal tracking, a far-away isolated line is not the current
    // path. In a recovery manoeuvre, the robot is deliberately searching, so
    // permit any visible candidate to re-establish the lock.
    if (cfg::kEnableLineSegmentLock && lineLockInitialised_ &&
        !allowDistantReacquire &&
        smallestDistance > cfg::kLineLockMaximumPositionJump) {
        return nullptr;
    }

    trackedPosition_ = candidate->position;
    lineLockInitialised_ = true;
    return candidate;
}

void Robot::resetLineLock() {
    trackedPosition_ = 0;
    lineLockInitialised_ = false;
}

bool Robot::junctionPresent(const SensorFrame& frame,
                            const LineSegment& trackedSegment) const {
    const bool trackedPathIsWide =
        trackedSegment.activeCount >= cfg::kJunctionActiveSensors;
    const bool trackedPathReachesEdge = trackedSegment.firstSensor == 0 ||
                                        trackedSegment.lastSensor == kViperSensorCount - 1;
    return frame.wide && trackedPathIsWide && trackedPathReachesEdge;
}

JunctionAction Robot::selectJunctionAction(const SensorFrame& frame) {
    JunctionAction action;
    if (returningFromDeadEnd_) {
        // The branch just returned from is behind the robot now. Pop its entry
        // and choose an available forward branch rather than replaying the route.
        returningFromDeadEnd_ = false;
        if (pathDepth_ > 0) {
            --pathDepth_;
        }
        action = firstAvailableAction(frame);
    } else {
        action = nextRouteAction(frame);
    }
    rememberJunctionAction(action);
    return action;
}

JunctionAction Robot::nextRouteAction(const SensorFrame& frame) {
    const JunctionAction requested = cfg::kRoute[routeIndex_ % cfg::kRouteLength];
    ++routeIndex_;
    return actionAvailable(requested, frame) ? requested : firstAvailableAction(frame);
}

JunctionAction Robot::firstAvailableAction(const SensorFrame& frame) const {
    // This order provides deterministic depth-first exploration after a return.
    if (actionAvailable(JunctionAction::LEFT, frame)) return JunctionAction::LEFT;
    if (actionAvailable(JunctionAction::STRAIGHT, frame)) return JunctionAction::STRAIGHT;
    if (actionAvailable(JunctionAction::RIGHT, frame)) return JunctionAction::RIGHT;
    return JunctionAction::STRAIGHT;
}

bool Robot::actionAvailable(JunctionAction action, const SensorFrame& frame) const {
    switch (action) {
        case JunctionAction::LEFT:
            return frame.leftEdge;
        case JunctionAction::RIGHT:
            return frame.rightEdge;
        case JunctionAction::STRAIGHT:
            return frame.line[6] >= cfg::kBranchThreshold ||
                   frame.line[7] >= cfg::kBranchThreshold ||
                   frame.line[8] >= cfg::kBranchThreshold ||
                   frame.line[9] >= cfg::kBranchThreshold;
    }
    return false;
}

void Robot::rememberJunctionAction(JunctionAction action) {
    if (pathDepth_ < cfg::kPathMemoryDepth) {
        pathMemory_[pathDepth_++].action = action;
    } else {
        // Retain the most recent path decisions; a tiny MCU cannot store a full
        // graph without odometry or identifiable markers at each intersection.
        memmove(pathMemory_, pathMemory_ + 1,
                sizeof(PathMemoryEntry) * (cfg::kPathMemoryDepth - 1));
        pathMemory_[cfg::kPathMemoryDepth - 1].action = action;
    }
}

void Robot::showStatus() const {
    Serial.print(F("state="));
    Serial.print(stateName());
    Serial.print(F(" calibrated="));
    Serial.print(sensors_.calibrationValid() ? F("yes") : F("no"));
    Serial.print(F(" position="));
    Serial.print(trackedPosition_);
    Serial.print(F(" full_position="));
    Serial.print(lastFrame_.position);
    Serial.print(F(" strength="));
    Serial.print(lastFrame_.strength);
    Serial.print(F(" active="));
    Serial.print(lastFrame_.activeCount);
    Serial.print(F(" segments="));
    Serial.print(lastFrame_.segmentCount);
    Serial.print(F(" polarity="));
    Serial.println(lastFrame_.polarity == LinePolarity::DARK ? F("dark") : F("light"));
}

void Robot::emitTelemetry() {
    if (!telemetryEnabled_ || millis() - lastTelemetryMs_ < cfg::kTelemetryIntervalMs) {
        return;
    }
    
    lastTelemetryMs_ = millis();
    Serial.print(F("p="));
    Serial.print(trackedPosition_);
    Serial.print(F(" g="));
    Serial.print(lastFrame_.position);
    Serial.print(F(" a="));
    Serial.print(lastFrame_.activeCount);
    Serial.print(F(" n="));
    Serial.print(lastFrame_.segmentCount);
    Serial.print(F(" q="));
    Serial.print(lastFrame_.strength);
    Serial.print(F(" m="));
    Serial.print(lastFrame_.polarity == LinePolarity::DARK ? 'D' : 'L');
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
        case RobotState::DEAD_END_TURN: return F("DEAD_END_TURN");
        case RobotState::FINISHED: return F("FINISHED");
        case RobotState::FAULT: return F("FAULT");
    }

    return F("UNKNOWN");
}
