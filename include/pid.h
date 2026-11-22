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
} PID;

void PIDInit(PID *pid, float kp, float ki, float kd);
float PIDUpdate(PID *pid, float setpoint, float measurement, float dt);
void PIDReset(PID *pid);

#endif
