#include "pid.h"
#include <stdlib.h>

void PIDInit(PID *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PIDReset(PID *pid) {
    if (pid == NULL) {
        return;
    }

    pid->previous_error = 0;
    pid->integral = 0;
}
