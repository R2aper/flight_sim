#include <rocketlib.h>

#define UPDATE_ROCKET_STATUS(r) (r)->update_status((r), (vec3_t){0, 0, _M_PI_2_}, calculate_forces)

/// @brief Calculate delta-v of rocket
double deltav(const rocket_t *r);

/// @brief Calculate if the rocket has enough delta-v for landing
bool is_enough_deltav(const rocket_t *r);

/// Describes forces that apply to the rocket
vec3_t calculate_forces(const rocket_t *r);

/// @brief Updates the rocket's state over a single time step using the Euler method(RK1)
void update_status_rk1(rocket_t *r, vec3_t new_directions,
                       vec3_t (*calculate_forces)(const rocket_t *));

/// @brief Updates the rocket's state over a single time step using the second-order Runge-Kutta
/// method(midpoint method)
void update_status_rk2(rocket_t *r, vec3_t new_directions,
                       vec3_t (*calculate_forces)(const rocket_t *));

/// @brief Updates the rocket's state over a single time step using the classic fourth-order
/// Runge-Kutta method
void update_status_rk4(rocket_t *r, vec3_t new_directions,
                       vec3_t (*calculate_forces)(const rocket_t *));

/// @brief Event detector for ground contact and other simulation events.
/// @return Returns the type of event detected.
event_type_t ground_contact_detector(rocket_t *current_state, const rocket_t *previous_state);

/// @brief Interpolates the rocket's state to the exact moment of a detected
/// event.
void hoverslam_event_interpolator(rocket_t *current_state, const rocket_t *previous_state,
                                  event_type_t event);

/// @brief Initializes the rocket state for a vertical fall scenario
rocket_t start_falling(double dt, double dry_mass, double fuel_mass, double height, engine_t engine,
                       planet_t pl);
