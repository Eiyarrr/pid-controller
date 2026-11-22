#ifndef PID_H

#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float previous_error;
    float integral;

    bool initialized;

    float output_max;
    float output_min;

    float integral_max;
    float integral_min;
} PID;

void PIDInit(PID *pid, float kp, float ki, float kd);
float PIDUpdate(PID *pid, float setpoint, float measurement, float dt);
void PIDReset(PID *pid);
void PIDSetOutputLimits(PID *pid, float upper_limit, float lower_limit);
void PIDSetIntegralLimits(PID *pid, float upper_limit, float lower_limit);

#endif
