#include <rocketlib/logger.h>
#include <rocketlib/rocket.h>
#define DISPLAY_STRIP_PREFIX
#include "common.h"

#include <assert.h>

double deltav(const rocket_t *r) {
  double u = calculate_u(r->engine), mass = FULL_MASS(*r), g = calculate_g(*r);

  return u * log(mass / (mass - r->fuel_mass)) - g * (r->fuel_mass / r->engine.consumption);
}

bool is_enough_deltav(const rocket_t *r) {
  // Max velocity = √2*g*h
  double max_v = sqrt(2 * calculate_g(*r) * r->coords.z);

  return (deltav(r) > max_v);
}

vec3_t calculate_forces(const void *r_ptr) {
  rocket_t *r = (rocket_t *)r_ptr;
  double mass = FULL_MASS(*r);
  return (vec3_t){0, 0, (CURRENT_THRUST(*r) - mass * calculate_g(*r)) / mass};
}

event_type_t ground_contact_detector(simulator_t *scene, const void *previous_state_ptr) {
  rocket_t *current_state = (rocket_t *)scene->object;
  const rocket_t *previous_state = (rocket_t *)previous_state_ptr;

  // Event 1: Ground contact
  if (current_state->coords.z <= 0 && previous_state->coords.z > 0) {
    return EV_GROUND_CONTACT;
  }

  // Event 2: Flying away (unstable behavior)
  if (current_state->velocity.z > 0 && current_state->time > 1.0) {
    current_state->velocity.z = INFINITY; // Mark as failure
    return EV_UNSTABLE;
  }

  return EV_NONE;
}

void hoverslam_event_interpolator(simulator_t *scene, const void *previous_state_ptr,
                                  event_type_t event) {
  rocket_t *current_state = (rocket_t *)scene->object;
  const rocket_t *previous_state = (rocket_t *)previous_state_ptr;

  if (event != EV_GROUND_CONTACT)
    return; // This interpolator only handles ground contact

  // Use linear interpolation to find the exact state at z=0
  // alpha is the fraction of the last time step before hitting the ground
  double alpha = previous_state->coords.z / (previous_state->coords.z - current_state->coords.z);

  // Interpolate state variables
  current_state->time = previous_state->time + alpha * scene->dt;
  current_state->coords.x =
      previous_state->coords.x + alpha * (current_state->coords.x - previous_state->coords.x);
  current_state->coords.y =
      previous_state->coords.y + alpha * (current_state->coords.y - previous_state->coords.y);
  current_state->velocity.x =
      previous_state->velocity.x + alpha * (current_state->velocity.x - previous_state->velocity.x);
  current_state->velocity.y =
      previous_state->velocity.y + alpha * (current_state->velocity.y - previous_state->velocity.y);
  current_state->velocity.z =
      previous_state->velocity.z + alpha * (current_state->velocity.z - previous_state->velocity.z);
  current_state->fuel_mass =
      previous_state->fuel_mass - alpha * (previous_state->fuel_mass - current_state->fuel_mass);

  // The final altitude is exactly zero
  current_state->coords.z = 0.0;
}

void update_status_rk1(simulator_t *scene, vec3_t new_directions,
                       vec3_t(calculate_forces)(const void *)) {
  rocket_t *r = (rocket_t *)scene->object;
  double dt = scene->dt;
  r->time += dt;
  r->directions = new_directions;

  r->acc = calculate_forces(r);

  r->velocity.x += r->acc.x * dt;
  r->velocity.y += r->acc.y * dt;
  r->velocity.z += r->acc.z * dt;

  r->coords.x += r->velocity.x * dt;
  r->coords.y += r->velocity.y * dt;
  r->coords.z += r->velocity.z * dt;

  r->fuel_mass -= r->engine.consumption * r->thrust_percent * dt;
  if (r->fuel_mass <= 0) { // Ran out of fuel
    r->fuel_mass = 0;
    CHANGE_THRUST(*r, 0);
  }
}

void update_status_rk2(simulator_t *scene, vec3_t new_directions,
                       vec3_t(calculate_forces)(const void *)) {
  rocket_t *r = (rocket_t *)scene->object;
  double dt = scene->dt;
  r->directions = new_directions;

  // --- Step 1: Calculate derivatives at the initial point (k1) ---
  vec3_t a1 = calculate_forces(r);
  vec3_t v1 = r->velocity;

  // --- Step 2: Evaluate the state of the system at the middle of the step (t +
  // dt/2) ---
  rocket_t k2 = *r;
  k2.coords.x += v1.x * (dt / 2.0);
  k2.coords.y += v1.y * (dt / 2.0);
  k2.coords.z += v1.z * (dt / 2.0);

  k2.velocity.x += a1.x * (dt / 2.0);
  k2.velocity.y += a1.y * (dt / 2.0);
  k2.velocity.z += a1.z * (dt / 2.0);

  k2.fuel_mass -= k2.engine.consumption * k2.thrust_percent * (dt / 2.0);
  k2.time += dt / 2.0;

  // --- Step 3: Calculate the derivatives at the midpoint (k2) ---
  vec3_t a2 = calculate_forces(&k2);
  vec3_t v2 = k2.velocity;

  // --- Step 4: Update the state using derivatives from the midpoint ---
  r->time += dt;
  r->acc = a2;
  r->coords.x += v2.x * dt;
  r->coords.y += v2.y * dt;
  r->coords.z += v2.z * dt;

  r->velocity.x += a2.x * dt;
  r->velocity.y += a2.y * dt;
  r->velocity.z += a2.z * dt;

  r->fuel_mass -= r->engine.consumption * r->thrust_percent * dt;
  if (r->fuel_mass < 0) {
    r->fuel_mass = 0;
    CHANGE_THRUST(*r, 0);
  }
}

void update_status_rk4(simulator_t *scene, vec3_t new_directions,
                       vec3_t(calculate_forces)(const void *)) {
  rocket_t *r = (rocket_t *)scene->object;
  double dt = scene->dt;
  r->directions = new_directions;

  rocket_t initial_state = *r;

  // --- k1: Derivatives at the initial point ---
  vec3_t v1 = initial_state.velocity;
  vec3_t a1 = calculate_forces(&initial_state);

  // --- k2: Derivatives at the midpoint (t + dt/2) using k1 ---
  rocket_t r_k2 = initial_state;
  r_k2.coords.x += v1.x * (dt / 2.0);
  r_k2.coords.y += v1.y * (dt / 2.0);
  r_k2.coords.z += v1.z * (dt / 2.0);
  r_k2.velocity.x += a1.x * (dt / 2.0);
  r_k2.velocity.y += a1.y * (dt / 2.0);
  r_k2.velocity.z += a1.z * (dt / 2.0);
  r_k2.fuel_mass -= initial_state.engine.consumption * initial_state.thrust_percent * (dt / 2.0);
  if (r_k2.fuel_mass < 0)
    r_k2.fuel_mass = 0;

  vec3_t v2 = r_k2.velocity;
  vec3_t a2 = calculate_forces(&r_k2);

  // --- k3: Derivatives at the midpoint (t + dt/2) using k2 ---
  rocket_t r_k3 = initial_state;
  r_k3.coords.x += v2.x * (dt / 2.0);
  r_k3.coords.y += v2.y * (dt / 2.0);
  r_k3.coords.z += v2.z * (dt / 2.0);
  r_k3.velocity.x += a2.x * (dt / 2.0);
  r_k3.velocity.y += a2.y * (dt / 2.0);
  r_k3.velocity.z += a2.z * (dt / 2.0);
  r_k3.fuel_mass -= initial_state.engine.consumption * initial_state.thrust_percent * (dt / 2.0);
  if (r_k3.fuel_mass < 0)
    r_k3.fuel_mass = 0;

  vec3_t v3 = r_k3.velocity;
  vec3_t a3 = calculate_forces(&r_k3);

  // --- k4: Derivatives at the endpoint (t + dt) using k3 ---
  rocket_t r_k4 = initial_state;
  r_k4.coords.x += v3.x * dt;
  r_k4.coords.y += v3.y * dt;
  r_k4.coords.z += v3.z * dt;
  r_k4.velocity.x += a3.x * dt;
  r_k4.velocity.y += a3.y * dt;
  r_k4.velocity.z += a3.z * dt;
  r_k4.fuel_mass -= initial_state.engine.consumption * initial_state.thrust_percent * dt;
  if (r_k4.fuel_mass < 0)
    r_k4.fuel_mass = 0;

  vec3_t v4 = r_k4.velocity;
  vec3_t a4 = calculate_forces(&r_k4);

  // --- Combine derivatives to update the state ---
  r->time += dt;

  // Update position using weighted average of velocities
  r->coords.x += (dt / 6.0) * (v1.x + 2.0 * v2.x + 2.0 * v3.x + v4.x);
  r->coords.y += (dt / 6.0) * (v1.y + 2.0 * v2.y + 2.0 * v3.y + v4.y);
  r->coords.z += (dt / 6.0) * (v1.z + 2.0 * v2.z + 2.0 * v3.z + v4.z);

  // Update velocity using weighted average of accelerations
  r->velocity.x += (dt / 6.0) * (a1.x + 2.0 * a2.x + 2.0 * a3.x + a4.x);
  r->velocity.y += (dt / 6.0) * (a1.y + 2.0 * a2.y + 2.0 * a3.y + a4.y);
  r->velocity.z += (dt / 6.0) * (a1.z + 2.0 * a2.z + 2.0 * a3.z + a4.z);

  // Update acceleration (for logging) as the weighted average
  r->acc.x = (a1.x + 2.0 * a2.x + 2.0 * a3.x + a4.x) / 6.0;
  r->acc.y = (a1.y + 2.0 * a2.y + 2.0 * a3.y + a4.y) / 6.0;
  r->acc.z = (a1.z + 2.0 * a2.z + 2.0 * a3.z + a4.z) / 6.0;

  // Update fuel mass (simple Euler is correct as consumption rate is constant
  // over the step)
  r->fuel_mass -= initial_state.engine.consumption * initial_state.thrust_percent * dt;
  if (r->fuel_mass < 0) {
    r->fuel_mass = 0;
    CHANGE_THRUST(*r, 0);
  }
}

rocket_t *start_falling(double dry_mass, double fuel_mass, double height, engine_t engine,
                        planet_t pl) {
  rocket_t *r = (rocket_t *)malloc(sizeof(rocket_t));
  if (!r) {
    return NULL;
  }

  r->d.display_fn = display_rocket;
  r->d.fdisplay_fn = fdisplay_rocket;
  r->d.sndisplay_fn = sndisplay_rocket;
  r->time = 0;
  r->velocity = VEC3_ZERO; // Start at top of trajectory
  r->acc = VEC3_ZERO;
  r->directions = (vec3_t){0, 0, _M_PI_2_};
  r->thrust_percent = 0;
  r->dry_mass = dry_mass;
  r->fuel_mass = fuel_mass;
  r->coords.x = 0;
  r->coords.y = 0;
  r->coords.z = height;
  r->engine = engine;
  r->pl = pl;

  return r;
}

void take_step(simulator_t *scene) {
  scene->time += scene->dt;
  scene->integrator(scene, (vec3_t){0, 0, _M_PI_2_}, calculate_forces);
}
