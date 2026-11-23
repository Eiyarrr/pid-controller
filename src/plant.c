#include "plant.h"
#include <stddef.h>

void PlantInit(Plant *plant, float angle) {
    plant->angle = angle;
    plant->angular_velocity = 0;
    plant->angular_acceleration = 0;

    plant->inertia = 0;
    plant->damping = 0;
}

void PlantUpdate(Plant *plant, float command, float dt) {
    if (plant == NULL) {
        return;
    }

    float new_angle = plant->angle + plant->angular_velocity * dt;
    float new_velocity =
        plant->angular_velocity + plant->angular_acceleration * dt;
    float new_acceleration =
        (command - plant->damping * plant->angular_velocity) / plant->inertia;

    plant->angle = new_angle;
    plant->angular_velocity = new_velocity;
    plant->angular_acceleration = new_acceleration;
}
