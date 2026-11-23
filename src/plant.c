#include "plant.h"
#include <stddef.h>

void PlantInit(Plant *plant, float angle) {
    plant->angle = angle;
    plant->angular_velocity = 0;

    plant->inertia = 0;
    plant->damping = 0;
}

void PlantUpdate(Plant *plant, float command, float dt) {
    if (plant == NULL) {
        return;
    }
    
    float acceleration =
        (command - plant->damping * plant->angular_velocity) / plant->inertia;

    float velocity =
        plant->angular_velocity + acceleration * dt;

    float angle = plant->angle + velocity * dt;

    plant->angle = angle;
    plant->angular_velocity = velocity;
}
