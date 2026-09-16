# Competition Line Follower — ESP32-S2 + Linea Viper X16

This is a non-blocking, competition-oriented line-following firmware for an **ESP32-S2** and a **Linea Viper X16** 16-element IR reflectance array. It replaces the earlier eight-sensor ESP32-S3 prototype with a calibrated 16-channel sensor pipeline, time-correct PID, curve-speed control, junction routing, bounded line recovery, persistent calibration, and a press-to-run safety state machine.

The active firmware is `src/main.cpp` and `src/robot/`; the original prototype files are retained for reference but excluded by `platformio.ini` (all under legacy/ folders).

## Competition features

- Per-sensor black/white calibration, smoothing, and a weighted line centroid from `-7500` (left) to `+7500` (right).
- PID with measured loop time, derivative filtering, bounded integral wind-up, and automatic speed reduction as curvature rises.
- Junction detection with a configurable route (`LEFT → RIGHT → STRAIGHT` by default). Turns continue until centre sensors reacquire the new line.
- Last-known-side recovery when the line vanishes; it faults after a bounded search instead of driving indefinitely.
- Calibration persisted in ESP32 NVS with a CRC check. A calibration is rejected if any X16 channel did not see both the line and floor.
- No `delay()` in racing states: sensor acquisition, PID, turn logic, recovery, and serial diagnostics all continue concurrently.

## Wiring architecture

The Linea Viper X16 uses a 74HC4067 analog multiplexer to support 16 IR sensors. The firmware drives the apparent onboard mux through four select lines and reads its common output:

```text
Viper X16's on-board 16:1 mux   SIG / OUT ── WEMOS S2 Mini GPIO 1 (ADC1_CH0)
                                S0..S3      ── WEMOS S2 Mini GPIO 2, 3, 4, 5

WEMOS GPIO 6,7,8      ── left H-bridge IN1, IN2, PWM/EN
WEMOS GPIO 9,10,11    ── right H-bridge IN1, IN2, PWM/EN
WEMOS GPIO 12         ── momentary START button to GND
WEMOS GPIO 15         ── built-in status LED
```

The pins are for the **WEMOS LOLIN S2 Mini** and are defined in [`RobotConfig.h`](include/RobotConfig.h). The board has native USB; the PlatformIO target is `lolin_s2_mini`.

### Electrical requirements — important

- The ESP32-S2 ADC and every X16-to-WEMOS signal must stay between **0 and 3.3 V**. Never connect a 5 V `SIG`/`OUT` line directly to GPIO 1. If the X16's interface operates at 5 V, use level shifting or an analogue divider only after confirming the module's documented signal topology.
- Do not infer the X16's supply voltage from its mux package. Confirm its `VCC`/`VIN` specification first; its IR emitters may have different power requirements from its logic output.
- Join sensor ground, ESP32 ground, and motor-driver logic ground at a low-impedance common point. Give motors a separate rated supply and add bulk capacitance near the driver. Motor current spikes move ADC readings and appear to the controller as false line movement.
- This driver expects 16 independent **analogue reflection outputs**. If a particular Viper X16 revision has serial, multiplexed, or threshold-only outputs, keep the controller and replace only `ViperX16::readChannel()` with that documented interface.

## Build, calibrate, run

The PlatformIO environment is `lolin_s2_mini`.

```powershell
platformio run
platformio run --target upload
platformio device monitor --baud 115200
```

1. Put the robot on the actual course surface and keep the wheels clear.
2. Hold **START** for one second. It sweeps left/right for 4.8 seconds, recording the minimum and maximum for every sensor.
3. Each sensor must see line and floor. The firmware refuses a calibration where any channel changes by less than `kMinimumCalibrationSpan`; that catches unplugged channels, wrong mux wiring, and an incomplete sweep.
4. Once `Calibration saved` appears, tap START to race. Console commands are `r` run, `c` calibrate, `x` emergency stop, `s` status, and `t` telemetry.

## Configuration and tuning

Edit [`RobotConfig.h`](include/RobotConfig.h):

- Set `kLineIsDark = false` if the line is more reflective than the surrounding floor. Internally, a strong line is always 1000.
- Measure the motor's static-friction threshold and set `kMotorMinimumPwm` just above it. A lower value makes the inside wheel hesitate; a higher one makes the robot hunt.
- Replace `kRoute` with the competition's junction sequence. Stop-line recognition is disabled by default because event marking conventions vary.
- Confirm that positive commands drive both wheels forward. Swap an H-bridge direction pair if not; do not compensate for a reversed wheel by changing PID signs.

Tune on the finished chassis, battery, tyres, sensor height, and course material:

1. Set `kPidI` and `kPidD` to zero, then raise `kPidP` until correction becomes decisive but begins to weave.
2. Raise `kPidD` to damp the weave. It responds to how quickly the line moves across the X16, which is especially valuable on fast corner entry. Too much turns sensor noise into motor chatter.
3. Add only enough `kPidI` to remove a constant bias such as unequal wheels. It is bounded deliberately, because stored correction is harmful at the steering limit.
4. Increase `kCruisePwm` gradually. If sharp corners fail, lower `kCornerPwm` or increase `kDerivativeSlowdown` before blindly increasing gain.
5. Telemetry reports centroid position (`p`), active channels (`a`), average line strength (`q`), normalised error (`e`), and requested left/right PWM (`l`/`r`). Log a difficult corner before changing thresholds.

## The kinematics behind the steering

The controller uses the centroid, not a single thresholded sensor. If the line appears on the right of the array, it increases left-wheel command and decreases right-wheel command. For a differential-drive chassis, the approximate yaw rate is:

```text
yaw rate ≈ (right wheel speed − left wheel speed) / track width
```

That relationship depends on actual wheel speed and track width, not PWM alone. PWM is an open-loop request: voltage sag, motor temperature, cornering load, and tyre slip all change the result. Wheel encoders plus an inner per-wheel velocity loop are the next meaningful upgrade if rules permit them; they make this outer line controller much more repeatable.

Mount the X16 rigidly, parallel to the floor, a little ahead of the axle. More forward offset creates useful look-ahead but magnifies position changes; too close to the axle delays the correction. Sensor height must be fixed—reflectance changes sharply with gap, so mechanical stiffness and calibration matter as much as PID values.

## Pre-race checklist

- Verify 3.3 V logic compatibility and a common logic ground.
- Calibrate on the exact mat, line colour, illumination, and ride height used in the event.
- Use telemetry to find flat, saturated, or weak sensor channels.
- Validate branch choices at low speed, then tune entry/turn durations for the wheelbase and intersection geometry.
- Test recovery and stop behaviour with the robot lifted first. Recovery intentionally faults after its timeout.
