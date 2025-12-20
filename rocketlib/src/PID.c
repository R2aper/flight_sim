#include "rocketlib/PID.h"

int display_pid(const void *self) {
  if (!self)
    return -1;

  PID *pid = (PID *)self;
  return printf("K_p = %f\nK_i = %f\nK_d = %f", pid->K_p, pid->K_i, pid->K_d);
}

int fdisplay_pid(const void *self, FILE *file) {
  if (!self || !file)
    return -1;

  PID *pid = (PID *)self;
  return fprintf(file, "K_p = %f\nK_i = %f\nK_d = %f", pid->K_p, pid->K_i, pid->K_d);
}

int sndisplay_pid(const void *self, char *buff, size_t size) {
  if (!self || !buff)
    return -1;

  PID *pid = (PID *)self;
  return snprintf(buff, size, "K_p = %f\nK_i = %f\nK_d = %f", pid->K_p, pid->K_i, pid->K_d);
}
