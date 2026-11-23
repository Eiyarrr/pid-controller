#ifndef PLANT_H

#define PLANT_H

typedef struct {
    float angle;
    float angular_velocity;

    float inertia;
    float damping;
} Plant;

void PlantInit(Plant *plant, float angle, float inertia, float damping);
void PlantUpdate(Plant *plant, float command, float dt);

#endif
