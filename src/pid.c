#include "pid.h"
#include <math.h>
#include <stddef.h>

static float clamp(float value, float min, float max) {
    return fmaxf(min, fminf(value, max));
}

void PIDInit(PID *pid, float kp, float ki, float kd) {
    if (pid == NULL || !isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0;
    pid->integral_max = 0;
    pid->integral_min = 0;

    pid->previous_error = 0;
    pid->previous_filtered_derivative = 0;
    pid->derivative_filter_alpha = 0;

    pid->output_max = 0;
    pid->output_min = 0;

    pid->initialized = false;
}

float PIDUpdate(PID *pid, float setpoint, float measurement, float dt) {
    // Ensure valid parameters
    if (pid == NULL || dt <= 0 || !isfinite(dt) || !isfinite(setpoint) ||
        !isfinite(measurement)) {
        return 0;
    }
    if (pid->output_min > pid->output_max ||
        pid->integral_min > pid->integral_max) {
        return 0;
    }

    // Calculate error and proportional contribution
    float current_error = setpoint - measurement;
    float p = pid->kp * current_error;

    // Calculate and clamp integral contribution
    float raw_integral = pid->integral + current_error * dt;
    pid->integral = clamp(raw_integral, pid->integral_min, pid->integral_max);
    float i = pid->ki * pid->integral;

    // Handle first update (no derivative history)
    if (!pid->initialized) {
        pid->previous_error = current_error;
        pid->initialized = true;
        float raw_output = p + i;
        return clamp(raw_output, pid->output_min, pid->output_max);
    }

    // Calculate and filter derivative contribution
    float raw_derivative = (current_error - pid->previous_error) / dt;
    float filtered_derivative =
        pid->derivative_filter_alpha * raw_derivative +
        (1 - pid->derivative_filter_alpha) * pid->previous_filtered_derivative;
    float d = pid->kd * filtered_derivative;

    pid->previous_filtered_derivative = filtered_derivative;
    pid->previous_error = current_error;

    float raw_output = p + i + d;
    return clamp(raw_output, pid->output_min, pid->output_max);
}

void PIDReset(PID *pid) {
    if (pid == NULL) {
        return;
    }

    pid->previous_error = 0;
    pid->integral = 0;
    pid->initialized = false;
    pid->previous_filtered_derivative = 0;
}

void PIDSetIntegralLimits(PID *pid, float min, float max) {
    if (pid == NULL || min > max || !isfinite(min) || !isfinite(max)) {
        return;
    }

    pid->integral_max = max;
    pid->integral_min = min;
}

void PIDSetDerivativeFilterAlpha(PID *pid, float derivative_filter_alpha) {
    if (pid == NULL || !isfinite(derivative_filter_alpha) ||
        derivative_filter_alpha > 1 || derivative_filter_alpha < 0) {
        return;
    }

    pid->derivative_filter_alpha = derivative_filter_alpha;
}

void PIDSetOutputLimits(PID *pid, float min, float max) {
    if (pid == NULL || min > max || !isfinite(min) || !isfinite(max)) {
        return;
    }

    pid->output_max = max;
    pid->output_min = min;
}
