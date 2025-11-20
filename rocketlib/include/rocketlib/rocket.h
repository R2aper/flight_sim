#ifndef ROCKET_H
#define ROCKET_H

/*
 * @file rocket.h
 * @brief Defines the data structures and interfaces for a simple rocket simulation
 *
 * This file contains the definitions for the rocket's physical properties
 * its engine, the planet it's launching from, and various utility macros
 * for calculations related to the simulation
 */

#include "../display.h"
#include "utils.h"

/// Header for rocket logger file
#define ROCKET_LOG_HEADER                                                                          \
  "time(s),dry_mass(kg),fuel_mass(kg),accOx(m/s^2),accOy(m/s^2),"                                  \
  "accOz(m/s^2),velocityOx(m/s),velocityOy(m/s),velocityOz(m/s),"                                  \
  "CoordinateOx(m),CoordinateOy(m),"                                                               \
  "CoordinateOz(m),"                                                                               \
  "thrust_percent(%)"

#define PRINT_ROCKET(r)                                                                            \
  {                                                                                                \
    r.d.self = &r;                                                                                 \
    clrscrn();                                                                                     \
    println("{}", &r);                                                                             \
    _sleep_(0.01);                                                                                 \
  }

typedef struct planet_t {
  double mass;   // 10^24 kg
  double radius; // km

} planet_t;

// #define calculate_g(r) 9.8
#define calculate_g(r) G *((r).pl.mass * 1e24 / pow((r).pl.radius * 1e3 + (r).coords.z, 2))

typedef struct engine_t {
  double thrust;      // N (Newtons)
  double consumption; // kg/s

} engine_t;

#define calculate_u(eng) (eng).thrust / (eng).consumption

typedef struct rocket_t {
  display_t d;
  engine_t engine;
  planet_t pl;
  vec3_t velocity;      // m/s
  vec3_t acc;           // m/s^2
  vec3_t coords;        // Coordinates (m)
  vec3_t directions;    // Angles (radians)
  double dt;            // s (delta time)
  double time;          // s (simulation time)
  double dry_mass;      // kg (mass without fuel)
  double fuel_mass;     // kg
  float thrust_percent; // Percent (0.0 to 1.0)

  /// @brief Updates the dynamics of the simulation after a time step 'dt'.
  void (*update_status)(struct rocket_t *, vec3_t new_directions,
                        vec3_t (*calculate_forces)(const struct rocket_t *));

  /// @brief Optional: Detects if a simulation event has occurred
  /// @return `true` if an event was detected, `false` otherwise
  bool (*event_detector)(struct rocket_t *current_state, const struct rocket_t *previous_state);

  /// @brief Optional: Interpolates the simulation state to the precise moment
  /// of a detected event
  void (*event_interpolator)(struct rocket_t *current_state, const struct rocket_t *previous_state);

} rocket_t;

#define FULL_MASS(rocket) (rocket).dry_mass + (rocket).fuel_mass
#define CURRENT_THRUST(rocket) (rocket).engine.thrust *(rocket).thrust_percent
#define CHANGE_THRUST(rocket, new_thrust) (rocket).thrust_percent = new_thrust

int display_rocket(const void *self);
int fdisplay_rocket(const void *self, FILE *file);
int sndisplay_rocket(const void *self, char *buff, size_t size);

#endif // ROCKET_H
