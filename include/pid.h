#ifndef PID_H

#define PID_H

typedef struct {
    double kp;
    double ki;
    double kd;

    double previous_error;
    double integral;
} PID;

#endif
