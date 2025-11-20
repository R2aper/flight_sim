#ifndef PID_H
#define PID_H

/*
 * @file PID.h
 * @brief A simple PID controller implementation
 *
 * This file defines the structures and functions for a Proportional-Integral-Derivative (PID)
 * controller
 */

#include "../display.h"

/**
 * @struct PID
 * @brief Represents a PID controller's state and parameters
 *
 */
typedef struct PID {
  display_t d;
  double P, I, D;
  double K_p, K_i, K_d;
  double integral, prev_err;

} PID;

int display_pid(const void *self);
int fdisplay_pid(const void *self, FILE *file);
int sndisplay_pid(const void *self, char *buff, size_t size);

#endif // PID_H