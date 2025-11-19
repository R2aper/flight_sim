#include "rocket.h"

int display_rocket(const void *self) {
  if (!self)
    return -1;

  rocket_t *r = (rocket_t *)self;
  return printf("Time: %.2f s\n"
                "Dry mass:%.2f kg\n"
                "Fuel mass:%.2f kg\n"
                "Acceleration(x):%.2f m/s\n"
                "Acceleration(y):%.2f m/s\n"
                "Acceleration(z):%.2f m/s\n"
                "Velocity(x):%.2f m/s\n"
                "Velocity(y):%.2f m/s\n"
                "Velocity(z):%.2f m/s\n"
                "Coordinate(x):%.2f m\n"
                "Coordinate(y):%.2f m\n"
                "Coordinate(z):%.2f m\n"
                "Thrust percent:%.2f%%\n",
                r->time, r->dry_mass, r->fuel_mass, r->acc.x, r->acc.y,
                r->acc.z, r->velocity.x, r->velocity.y, r->velocity.z,
                r->coords.x, r->coords.y, r->coords.z, r->thrust_percent * 100);
}

int fdisplay_rocket(const void *self, FILE *file) {
  if (!self || !file)
    return -1;

  rocket_t *r = (rocket_t *)self;
  return fprintf(
      file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
      r->time, r->dry_mass, r->fuel_mass, r->acc.x, r->acc.y, r->acc.z,
      r->velocity.x, r->velocity.y, r->velocity.z, r->coords.x, r->coords.y,
      r->coords.z, r->thrust_percent * 100);
}

int sndisplay_rocket(const void *self, char *buff, size_t size) {
  if (!self || !buff)
    return -1;

  rocket_t *r = (rocket_t *)self;
  return snprintf(
      buff, size,
      "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
      r->time, r->dry_mass, r->fuel_mass, r->acc.x, r->acc.y, r->acc.z,
      r->velocity.x, r->velocity.y, r->velocity.z, r->coords.x, r->coords.y,
      r->coords.z, r->thrust_percent * 100);
}