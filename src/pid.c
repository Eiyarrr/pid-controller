#include "pid.h"
#include <math.h>
#include <stddef.h>

void PIDInit(PID *pid, float kp, float ki, float kd) {
    if (pid == NULL || !isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0;
    pid->integral = 0;
    pid->initialized = false;
    pid->output_max = 0;
    pid->output_min = 0;
    pid->integral_max = 0;
    pid->integral_min = 0;
    pid->previous_derivative = 0;
    pid->derivative_smoothing = 0;
}

float PIDUpdate(PID *pid, float setpoint, float measurement, float dt) {
    if (pid == NULL || dt <= 0 || !isfinite(dt)) {
        return 0;
    }
    if (pid->output_min > pid->output_max ||
        pid->integral_min > pid->integral_max) {
        return 0;
    }

    float current_error = setpoint - measurement;
    float p = pid->kp * current_error;

    float integral = pid->integral + current_error * dt;
    float clamped_integral =
        fmax(pid->integral_min, fmin(integral, pid->integral_max));
    pid->integral = clamped_integral;
    float i = pid->ki * pid->integral;

    if (!pid->initialized) {
        pid->previous_error = current_error;
        pid->initialized = true;
        float output = p + i;
        float clamped_output =
            fmaxf(pid->output_min, fminf(output, pid->output_max));
        return clamped_output;
    }

    float raw_derivative = (current_error - pid->previous_error) / dt;
    float filtered_derivative =
        pid->derivative_smoothing +
        (1 - pid->derivative_smoothing) * pid->previous_derivative;
    float d = pid->kd * filtered_derivative;
    pid->previous_derivative = filtered_derivative;

    pid->previous_error = current_error;

    float output = p + i + d;
    float clamped_output =
        fmaxf(pid->output_min, fminf(output, pid->output_max));
    return clamped_output;
}

void PIDReset(PID *pid) {
    if (pid == NULL) {
        return;
    }

    pid->previous_error = 0;
    pid->integral = 0;
    pid->initialized = false;
}

void PIDSetOutputLimits(PID *pid, float min, float max) {
    if (pid == NULL || min > max || !isfinite(min) || !isfinite(max)) {
        return;
    }

    pid->output_max = max;
    pid->output_min = min;
}

void PIDSetIntegralLimits(PID *pid, float min, float max) {
    if (pid == NULL || min > max || !isfinite(min) || !isfinite(max)) {
        return;
    }

    pid->integral_max = max;
    pid->integral_min = min;
}

void PIDSetDerivativeSmoothing(PID *pid, float derivative_smoothing) {
    if (pid == NULL || !isfinite(derivative_smoothing)) {
        return;
    }

    pid->derivative_smoothing = derivative_smoothing;
}
