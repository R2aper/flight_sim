#include <rocketlib.h>



/// @brief Calculate delta-v of rocket
double deltav(const rocket_t *r);

/// @brief Calculate if the rocket has enough delta-v for landing
bool is_enough_deltav(const rocket_t *r);

/// Describes forces that apply to the rocket
vec3_t calculate_forces(const void *r_ptr);

/// @brief Updates the rocket's state over a single time step using the Euler method(RK1)
void update_status_rk1(simulator_t *scene, vec3_t new_directions,
                       vec3_t(calculate_forces)(const void *));

/// @brief Updates the rocket's state over a single time step using the second-order Runge-Kutta
/// method(midpoint method)
void update_status_rk2(simulator_t *scene, vec3_t new_directions,
                       vec3_t(calculate_forces)(const void *));

/// @brief Updates the rocket's state over a single time step using the classic fourth-order
/// Runge-Kutta method
void update_status_rk4(simulator_t *scene, vec3_t new_directions,
                       vec3_t(calculate_forces)(const void *));

/// @brief Event detector for ground contact and other simulation events.
/// @return Returns the type of event detected.
event_type_t ground_contact_detector(simulator_t *scene, const void *previous_state_ptr);

/// @brief Interpolates the rocket's state to the exact moment of a detected
/// event.
void hoverslam_event_interpolator(simulator_t *scene, const void *previous_state_ptr,
                                  event_type_t event);

void take_step(simulator_t *scene);

/// @brief Initializes the rocket state for a vertical fall scenario
rocket_t *start_falling(double dry_mass, double fuel_mass, double height, engine_t engine,
                        planet_t pl);

#define rocket_free(r) free(r)
