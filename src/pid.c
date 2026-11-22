#include "pid.h"
#include <stddef.h>

void PIDInit(PID *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
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
}

float PIDUpdate(PID *pid, float setpoint, float measurement, float dt) {
    if (pid == NULL || dt <= 0) {
        return 0;
    }

    float current_error = setpoint - measurement;
    float p = pid->kp * current_error;

    pid->integral += current_error * dt;
    float i = pid->ki * pid->integral;

    if (!pid->initialized) {
        pid->previous_error = current_error;
        pid->initialized = true;
        return p + i;
    }

    float derivative = (current_error - pid->previous_error) / dt;
    float d = pid->kd * derivative;

    pid->previous_error = current_error;

    return p + i + d;
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
    if (pid == NULL || min > max) {
        return;
    }

    pid->output_max = max;
    pid->output_min = min;
}

void PIDSetIntegralLimits(PID *pid, float min, float max) {
    if (pid == NULL || min > max) {
        return;
    }

    pid->integral_max = max;
    pid->integral_min = min;
}
