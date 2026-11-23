#include "pid.h"
#include "plant.h"
#include "tests.h"

#include <stdlib.h>

void run_simulation() {
    float kp = 0;
    float ki = 0;
    float kd = 0;

    float starting_angle = 0;
    float inertia = 0;
    float damping = 0;

    float setpoint = 0;
    float dt = 0;

    int max_loops = 0;

    PID *pid = malloc(sizeof(PID));
    PIDInit(pid, kp, ki, kd);

    Plant *plant = malloc(sizeof(Plant));
    PlantInit(plant, starting_angle, inertia, damping);

    float measurement;
    float command;

    for (int i = 0; i < max_loops; i++) {
        measurement = plant->angle;
        command = PIDUpdate(pid, setpoint, measurement, dt);
        PlantUpdate(plant, command, dt);
    }

    free(pid);
    free(plant);
}
