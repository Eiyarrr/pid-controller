#include "pid.h"
#include "plant.h"
#include "tests.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void run_simulation() {
    float kp = 0.02;
    float ki = 0.0003;
    float kd = 0.003;

    float min_integral = -0.01;
    float max_integral = 0.01;

    float min_output = -0.003;
    float max_output = 0.003;

    float derivative_smoothing_alpha = 1;

    float starting_angle = 0;
    float inertia = 0.00015;
    float damping = 0.00002;

    float setpoint = M_PI / 2;
    float dt = 0.01;

    int max_loops = 1000;

    PID *pid = malloc(sizeof(PID));
    if (pid == NULL) {
        printf("failed to malloc pid\n");
        return;
    }
    PIDInit(pid, kp, ki, kd);
    PIDSetIntegralLimits(pid, min_integral, max_integral);
    PIDSetOutputLimits(pid, min_output, max_output);
    PIDSetDerivativeFilterAlpha(pid, derivative_smoothing_alpha);

    Plant *plant = malloc(sizeof(Plant));
    if (plant == NULL) {
        printf("failed to malloc plant\n");
        free(pid);
        return;
    }
    PlantInit(plant, starting_angle, inertia, damping);

    FILE *file = fopen("tests/simulation_results", "w");
    if (file == NULL) {
        printf("failed to open file\n");
        free(pid);
        free(plant);
        return;
    }
    fprintf(file, "setpoint: %f | kp/ki/kd: %f/%f/%f | inertia: %f | damping: %f\n", setpoint, kp, ki, kd, inertia, damping);

    float measurement;
    float command;

    printf("starting simulation\n");
    for (int i = 0; i < max_loops; i++) {
        measurement = plant->angle;
        command = PIDUpdate(pid, setpoint, measurement, dt);
        PlantUpdate(plant, command, dt);

        fprintf(file, "dt: %f | cmd: %f | angle: %f | off: %f\n", dt * i,
                command, plant->angle, setpoint - plant->angle);
    }

    free(pid);
    free(plant);

    fflush(file);
    fclose(file);

    printf("finished simulation\n");
}
