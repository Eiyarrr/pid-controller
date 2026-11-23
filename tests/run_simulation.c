#include "pid.h"
#include "plant.h"
#include "tests.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void run_simulation() {
    float kp = 0;
    float ki = 0;
    float kd = 0;

    float min_integral = -10;
    float max_integral = 10;

    float min_output = -10;
    float max_output = 10;

    float starting_angle = 0;
    float inertia = 10;
    float damping = 0;

    float setpoint = M_PI / 2;
    float dt = 0.01;

    int max_loops = 500;

    PID *pid = malloc(sizeof(PID));
    if (pid == NULL) {
        printf("failed to malloc pid\n");
        return;
    }
    PIDInit(pid, kp, ki, kd);
    PIDSetIntegralLimits(pid, min_integral, max_integral);
    PIDSetOutputLimits(pid, min_output, max_output);

    Plant *plant = malloc(sizeof(Plant));
    if (plant == NULL) {
        printf("failed to malloc plant\n");
        return;
    }
    PlantInit(plant, starting_angle, inertia, damping);

    FILE *file = fopen("tests/simulation_results", "w");
    if (file == NULL) {
        printf("failed to open file\n");
        return;
    }

    float measurement;
    float command;

    printf("starting simulation\n");
    for (int i = 0; i < max_loops; i++) {
        fprintf(file, "%f : %f\n", dt * i, plant->angle);

        measurement = plant->angle;
        command = PIDUpdate(pid, setpoint, measurement, dt);
        PlantUpdate(plant, command, dt);
    }

    free(pid);
    free(plant);
    fclose(file);

    printf("finished simulation\n");
}
