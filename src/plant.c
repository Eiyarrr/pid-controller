#include "plant.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

static bool PlantIsValid(Plant *plant) {
    if (plant == NULL || plant->inertia <= 0 || !isfinite(plant->angle) ||
        !isfinite(plant->angular_velocity) || !isfinite(plant->inertia) ||
        !isfinite(plant->damping)) {
        return false;
    }

    return true;
}

void PlantInit(Plant *plant, float angle) {
    if (plant == NULL) {
        return;
    }

    plant->angle = angle;
    plant->angular_velocity = 0;

    plant->inertia = 0;
    plant->damping = 0;
}

void PlantUpdate(Plant *plant, float command, float dt) {
    if (!PlantIsValid(plant) || dt < 0 || !isfinite(dt) || !isfinite(command)) {
        return;
    }

    float acceleration =
        (command - plant->damping * plant->angular_velocity) / plant->inertia;

    float velocity = plant->angular_velocity + acceleration * dt;

    float angle = plant->angle + velocity * dt;

    plant->angle = angle;
    plant->angular_velocity = velocity;
}
