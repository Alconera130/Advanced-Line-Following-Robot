#include "LineController.h"

#include "RobotConfig.h"

void LineController::reset() {
    previousError_ = 0.0F;
    integral_ = 0.0F;
    filteredDerivative_ = 0.0F;
}

ControlOutput LineController::update(int16_t position, float dtSeconds) {
    const float dt = constrain(dtSeconds, 0.001F, 0.050F);
    const float error = constrain(position / static_cast<float>(kPositionLimit), -1.0F, 1.0F);
    const float rawDerivative = (error - previousError_) / dt;
    filteredDerivative_ = cfg::kDerivativeFilter * filteredDerivative_ +
                            (1.0F - cfg::kDerivativeFilter) * rawDerivative;

    // The integral is bounded in normalised-error seconds. This prevents the
    // controller from storing a large correction while a tight turn saturates it.
    integral_ = constrain(integral_ + error * dt, -cfg::kIntegralLimit,
                            cfg::kIntegralLimit);
    float correction = cfg::kPidP * error + cfg::kPidI * integral_ +
                        cfg::kPidD * filteredDerivative_;
    correction = constrain(correction, -static_cast<float>(cfg::kMaxCorrection),
                            static_cast<float>(cfg::kMaxCorrection));

    const float curvature = constrain(fabsf(error) +
                                        cfg::kDerivativeSlowdown * fabsf(filteredDerivative_),
                                    0.0F, 1.0F);
    const int16_t cruise = static_cast<int16_t>(cfg::kCruisePwm -
        curvature * (cfg::kCruisePwm - cfg::kCornerPwm));
    previousError_ = error;

    ControlOutput output{};
    output.error = error;
    output.correction = static_cast<int16_t>(correction);
    output.cruise = cruise;
    output.left = constrain(static_cast<int16_t>(cruise + output.correction),
                            -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm);
    output.right = constrain(static_cast<int16_t>(cruise - output.correction),
                            -cfg::kMotorMaxPwm, cfg::kMotorMaxPwm);
    return output;
}
