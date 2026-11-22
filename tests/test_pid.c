#include "pid.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    PID *pid = malloc(sizeof(PID));
    float kp = 0;
    float ki = 0;
    float kd = 0;

    PIDInit(pid, kp, ki, kd);

    float setpoint = 50;
    float measurement = 0;
    float dt = 0.5;
    float command = PIDUpdate(pid, setpoint, measurement, dt);
    if (command != 0) {
        printf("expected command of 0, got %f\n", command);
    }

    pid->kp = 1;
    measurement += command;
    dt = 0.3;
    command = PIDUpdate(pid, setpoint, measurement, dt);
    if (command != 50) {
        printf("expected command of 50, got %f\n", command);
    }

    dt = 1;
    measurement += command;
    command = PIDUpdate(pid, setpoint, measurement, dt);
    if (command != 0) {
        printf("expected command of 0 (1), got %f\n", command);
    }

    PIDReset(pid);
    pid->kp = 0;
    pid->ki = 1;
    measurement = 0;
    dt = 0.1;
    command = PIDUpdate(pid, setpoint, measurement, dt);
    if (command != 5) {
        printf("expected command of 5, got %f\n", command);
    }

    measurement += command;
    command = PIDUpdate(pid, setpoint, measurement, dt);
    if (command != 9.5) {
        printf("expected command of 9.5, got %f\n", command);
    }

    PIDReset(pid);
    pid->ki = 0;
    pid->kd = 1;
    measurement = 0;
    dt = 0.1;
    command = PIDUpdate(pid, setpoint, measurement, dt);
    if (command != 0) {
        printf("expected command of 0, got %f\n", command);
    }

    measurement += command;
    command = PIDUpdate(pid, setpoint, measurement, dt);
    if (command != 0) {
        printf("expected command of 0, got %f\n", command);
    }

    printf("tests completed\n");
    return 0;
}
