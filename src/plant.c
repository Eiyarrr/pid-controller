#include "plant.h"

void PlantInit(Plant *plant, float angle) {
    plant->angle = angle;
    plant->angular_velocity = 0;
    plant->angular_acceleration = 0;

    plant->inertia = 0;
    plant->damping = 0;
}
