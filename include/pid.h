#ifndef PID_H

#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float previous_error;
    float integral;
    float integral_max;
    float integral_min;

    float previous_filtered_derivative;
    float derivative_filter_alpha;

    float output_max;
    float output_min;

    bool initialized;
} PID;

void PIDInit(PID *pid, float kp, float ki, float kd);
float PIDUpdate(PID *pid, float setpoint, float measurement, float dt);
void PIDReset(PID *pid);

void PIDSetIntegralLimits(PID *pid, float min, float max);
void PIDSetDerivativeSmoothing(PID *pid, float derivative_smoothing);
void PIDSetOutputLimits(PID *pid, float min, float max);

#endif
