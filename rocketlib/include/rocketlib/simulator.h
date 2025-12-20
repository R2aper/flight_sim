#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "events.h"
#include "utils.h"

/**
 * TODO
 *
 */

typedef struct simulator_t {
  double dt;    // Step
  double time;  // Time passed since start simulation
  void *object; // Pointer to simulated object

  void (*integrator)(struct simulator_t *, vec3_t new_directions,
                     vec3_t(calc_forces)(const void *));

  /// @brief Optional: Detects if a simulation event has occurred.
  /// @return The type of event detected (`event_type_t`)
  event_type_t (*event_detector)(struct simulator_t *, const void *prev);

  /// @brief Optional: Interpolates the simulation state to the precise moment
  /// of a detected event
  void (*event_interpolator)(struct simulator_t *, const void *prev, event_type_t event);

  void (*take_step)(struct simulator_t *);

} simulator_t;

#endif // SIMULATOR_H
