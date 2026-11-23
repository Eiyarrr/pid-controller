#ifndef PLANT_H

#define PLANT_H

typedef struct {
    float angle;
    float angular_velocity;
    float angluar_acceleration;

    float inertia;
    float damping;
} Plant;

#endif
