#define DISPLAY_IMPLEMENTATION
#define DISPLAY_STRIP_PREFIX
#include "common.h"
#include <rocketlib/PID.h>

#include <assert.h>

typedef struct result_t {
  rocket_t r;
  PID pid;
  int it;

} result_t;

/// @brief Calculate the required thrust percentage using a PID controller.
/// The target velocity is based on the principle of a gravity turn, where the
/// ideal velocity at a given altitude matches the free-fall velocity. The
/// controller will apply thrust to reduce speed when the rocket exceeds this
/// target velocity
/// @return Percent of thrust
double pid_calculate_thrust(PID *pid, rocket_t *r, double dt) {
  // Target velocity is the velocity a body would have in free fall from the
  // current altitude. This profile means the rocket falls freely until its
  // speed exceeds this value, at which point the engine brakes to maintain the
  // profile
  double target_velocity = -sqrt(2 * calculate_g(*r) * r->coords.z);
  double err = target_velocity - r->velocity.z;

  pid->P = pid->K_p * err;
  pid->integral += err * dt;
  pid->I = pid->K_i * pid->integral;
  pid->D = pid->K_d * (err - pid->prev_err) / dt;

  double thrust = pid->P + pid->I + pid->D;
  thrust = MAX(0, MIN(thrust, r->engine.thrust));

  pid->prev_err = err;

  return (thrust) / r->engine.thrust;
}

/// @brief Calculate the cost for the PID tuning algorithm
/// It simulates the rocket landing and returns a cost value representing the
/// landing quality. A lower cost is better. The goal is to have the final
/// velocity and altitude as close to zero as possible.
/// @return Cost
double evaluate_pid_cost(PID *pid, simulator_t scene, double weights[3]) {
  pid->integral = 0;
  pid->prev_err = 0;
  rocket_t *r = (rocket_t *)scene.object;
  double initial_fuel_mass = r->fuel_mass;

  event_type_t event = EV_NONE;
  rocket_t prev_state;
  while (event != EV_GROUND_CONTACT) {
    prev_state = *r;

    double desired_thrust = pid_calculate_thrust(pid, r, scene.dt);
    if (desired_thrust > 0 && r->fuel_mass > 0)
      CHANGE_THRUST(*r, MIN(1, desired_thrust));
    else
      CHANGE_THRUST(*r, 0);

    scene.take_step(&scene);
    event = scene.event_detector(&scene, &prev_state);

    if (event == EV_UNSTABLE || r->coords.z <= 0)
      break;
  }

  scene.event_interpolator(&scene, &prev_state, event);

  double fuel_used = initial_fuel_mass - r->fuel_mass;

  // Cost is a combination of final velocity, how far from the ground it and how
  // much fuel we used
  return weights[0] * fabs(r->velocity.z) + weights[1] * fabs(r->coords.z) + weights[2] * fuel_used;
}

/// @brief An implementation of the Twiddle algorithm for auto-tuning PID
/// coefficients
/// It systematically adjusts the Kp, Ki, and Kd values to minimize the cost
/// returned by evaluate_pid_cost
/// @return Optimized PID
PID tune_pid_twiddle(simulator_t scene, double tolerance, double weights[3], double dp[3]) {
  PID pid = {0};
  pid.d.display_fn = display_pid;
  pid.d.self = &pid;

  double p[] = {pid.K_p, pid.K_i, pid.K_d};

  rocket_t initial_rocket_state = *(rocket_t *)scene.object;
  double initial_scene_time = scene.time;

  double best_err = evaluate_pid_cost(&pid, scene, weights);

  *(rocket_t *)scene.object = initial_rocket_state;
  scene.time = initial_scene_time;

  while ((dp[0] + dp[1] + dp[2]) > tolerance) {
    for (int i = 0; i < 3; i++) {
      p[i] += dp[i];
      pid.K_p = p[0];
      pid.K_i = p[1];
      pid.K_d = p[2];
      double err = evaluate_pid_cost(&pid, scene, weights);

      *(rocket_t *)scene.object = initial_rocket_state;
      scene.time = initial_scene_time;

      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      } else {
        p[i] -= 2 * dp[i];
        pid.K_p = p[0];
        pid.K_i = p[1];
        pid.K_d = p[2];
        err = evaluate_pid_cost(&pid, scene, weights);

        *(rocket_t *)scene.object = initial_rocket_state;
        scene.time = initial_scene_time;

        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }

  pid.K_p = p[0];
  pid.K_i = p[1];
  pid.K_d = p[2];

  return pid;
}

/// @brief Simulate landing using an optimized PID controller
/// It first calculates the optimized parameters for the PID using
/// tune_pid_twiddle
/// @param tolerance Precision for the tuning algorithm
/// @param print Print data during flight?
/// @param log Log simulation data to a file?
/// @return The struct of tuned PID controller, rocket stats after land and
/// number of iterations during simulation
result_t pid_landing_simulation(simulator_t *scene, double tolerance, double weights[3],
                                double dp[3], bool print, bool log) {
  logger_t l = (logger_t){NULL, NULL};
  if (log) {
    l = logger_init("pid_flight_sim.csv");
    assert(l.file);
    fprintln(l.file, ROCKET_LOG_HEADER);
  }

  PID pid = tune_pid_twiddle(*scene, tolerance, weights, dp);
  pid.integral = 0;
  pid.prev_err = 0;

  rocket_t *r = (rocket_t *)scene->object;

  int it = 0;
  event_type_t event = EV_NONE;
  rocket_t prev_state;
  while (event != EV_GROUND_CONTACT) {
    it++;
    prev_state = *r;

    double desired_thrust = pid_calculate_thrust(&pid, r, scene->dt);

    if (desired_thrust > 0 && r->fuel_mass > 0)
      CHANGE_THRUST(*r, MIN(1, desired_thrust));
    else
      CHANGE_THRUST(*r, 0);

    scene->take_step(scene);
    event = scene->event_detector(scene, &prev_state);

    if (log && is_almost_integer(r->time, 0.01))
      logger_write_rocket(&l, r);

    if (print)
      PRINT_ROCKET(*r);

    if (event == EV_UNSTABLE || r->coords.z <= 0)
      break;
  }

  scene->event_interpolator(scene, &prev_state, event);

  if (l.file)
    logger_free(&l);

  return (result_t){*r, pid, it};
}

void usage() {
  puts("OPTIONS:\n"
       "--print\t\t\tPrint simulation\n"
       "--log\t\t\tLog simulation into cvs file\n"
       "--rocket <file>\t\tSpecify file with simulation parameters\n"
       "--dt <number>\t\tChange dt variable(default is 2e-3)\n"
       "--tolerance <number>\tChange tolerance variable(default is 1e-4)\n"
       "-h\t\t\tPrint this help message");
}

int main(int argc, char *argv[]) {
  double dt = 2e-3, tolerance = 1e-4;
  double dp[3], weights[3];
  bool to_print = false, to_log = false;
  double fuel_mass = 0.0, dry_mass = 0.0, altitude = 0.0;
  simulator_t scene = {0};
  engine_t eng = {0};
  planet_t pl = {0};
  char *rocket_file = "rocket.dat";

  // Parsing cmd args
  for (int i = 1; i < argc; i++) {
    char *token = argv[i];
    if (strcmp(token, "-h") == 0) {
      usage();
      return 0;
    } else if (strcmp(token, "--dt") == 0) {
      if (!argv[i + 1]) {
        fprintln(stderr, "Expected value after '%s'!", token);
        return -1;
      }
      if ((dt = atof(argv[++i])) == 0.0) {
        fprintln(stderr, "Invalid value : %s", argv[i]);
        return -1;
      }
    } else if (strcmp(token, "--tolerance") == 0) {
      if (!argv[i + 1]) {
        fprintln(stderr, "Expected value after '%s'!", token);
        return -1;
      }
      if ((tolerance = atof(argv[++i])) == 0.0) {
        fprintln(stderr, "Invalid value : %s", argv[i]);
        return -1;
      }
    } else if (strcmp(token, "--rocket") == 0) {
      if (!argv[i + 1]) {
        fprintln(stderr, "Expected value after '%s'!", token);
        return -1;
      }
      rocket_file = argv[++i];
    } else if (strcmp(token, "--print") == 0)
      to_print = true;
    else if (strcmp(token, "--log") == 0)
      to_log = true;
    else {
      println("Unknown flag: %s", token);
      return -1;
    }
  }

  // Parsing 'rocket_file' file
  fparser_t fp = fparser_init(rocket_file);
  if (!fp.file) {
    fprintln(stderr, "No '%s' file was found!", rocket_file);
    return -1;
  }

  fparser_parse(&fp);
  fparser_free(&fp);

  pl.mass = fparser_get_var(&fp, "planet", "mass").value;
  pl.radius = fparser_get_var(&fp, "planet", "radius").value;

  eng.thrust = fparser_get_var(&fp, "engine", "thrust").value;
  eng.consumption = fparser_get_var(&fp, "engine", "consumption").value;

  fuel_mass = fparser_get_var(&fp, "rocket", "fuel_mass").value;
  dry_mass = fparser_get_var(&fp, "rocket", "dry_mass").value;
  altitude = fparser_get_var(&fp, "rocket", "altitude").value;

  weights[0] = fparser_get_var(&fp, "pid_weights", "velocity").value;
  weights[1] = fparser_get_var(&fp, "pid_weights", "altitude").value;
  weights[2] = fparser_get_var(&fp, "pid_weights", "fuel").value;

  if (weights[0] == 0 && weights[1] == 0 && weights[2] == 0) {
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.1;
    println("Invalid pid_weights! Setting default ones:\nVelocity: "
            "%f\nAltitude: %f\nFuel: %f",
            weights[0], weights[1], weights[2]);
  }

  dp[0] = fparser_get_var(&fp, "pid_start_values", "K_p").value;
  dp[1] = fparser_get_var(&fp, "pid_start_values", "K_i").value;
  dp[2] = fparser_get_var(&fp, "pid_start_values", "K_d").value;

  if (dp[0] == 0 && dp[1] == 0 && dp[2] == 0) {
    dp[0] = 10.;
    dp[1] = 5.0;
    dp[2] = 1.0;
    println("Invalid pid_start_values! Setting default ones:\nK_p: %f\nK_i: "
            "%f\nK_d: %f",
            dp[0], dp[1], dp[2]);
  }

  rocket_t *r = start_falling(dry_mass, fuel_mass, altitude, eng, pl);
  assert(r);
  r->d.self = r;

  if (!is_enough_deltav(r)) {
    println("Available delta-v: %.2f\nNot enough for landing!", deltav(r));
    rocket_free(r);
    return -1;
  }

  scene.dt = dt;
  scene.integrator = update_status_rk4;
  scene.event_detector = ground_contact_detector;
  scene.event_interpolator = hoverslam_event_interpolator;
  scene.object = r;
  scene.take_step = take_step;

  result_t result = pid_landing_simulation(&scene, tolerance, weights, dp, to_print, to_log);

  result.r.d.self = &result.r;
  result.pid.d.self = &result.pid;
  println("Rocket stats after land:\n{}\nTuned PID:\n{}\nTotal "
          "iterations during simulation:%d",
          &result.r, &result.pid, result.it);

  rocket_free(r);

  return 0;
}
