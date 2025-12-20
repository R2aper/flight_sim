#include <rocketlib/logger.h>
#include <rocketlib/simulator.h>
#define DISPLAY_IMPLEMENTATION
#define DISPLAY_STRIP_PREFIX
#include "common.h"

#include <assert.h>

typedef struct result_t {
  rocket_t r;
  double time_to_burn;
  int it;

} result_t;

/// @brief Simulate a flight where the engine ignites after a specified time
/// Used for calculating the time of a hoverslam in
/// golden_search_hoverslam
/// @return Velocity at landing/crash
double velocity_at_landing(simulator_t *scene, double ignition_time) {
  event_type_t event = EV_NONE;
  rocket_t prev;
  rocket_t *r = (rocket_t *)scene->object;

  while (event != EV_GROUND_CONTACT) {
    if (scene->time >= ignition_time && r->thrust_percent == 0)
      CHANGE_THRUST(*r, 1);

    prev = *r;
    scene->take_step(scene);
    event = scene->event_detector(scene, &prev);

    if (event == EV_UNSTABLE || r->coords.z <= 0)
      break;
  }

  scene->event_interpolator(scene, &prev, event);

  double result = fabs(r->velocity.z);

  scene->time = 0;

  return result;
}

/// @brief Find the best time to ignite the engine using a golden-section
/// search. Treats velocity_at_landing as a function to be minimized
/// @param eps Precision
/// @return Time to start the engine
double golden_search_hoverslam(simulator_t *scene, double eps) {
  rocket_t r = *(rocket_t *)scene->object;
  double time = scene->time, dt = scene->dt;

  double g = calculate_g(r), phi = (1 + sqrt(5)) / 2; // φ ≈ 1.618
  double right = sqrt((r.coords.z * 2) / g), left = 0;
  double m1 = right - (right - left) / phi, m2 = left + (right - left) / phi;
  double f_m1 = velocity_at_landing(scene, m1);
  scene->dt = dt;
  scene->time = time;
  *(rocket_t *)scene->object = r;

  double f_m2 = velocity_at_landing(scene, m2);
  scene->dt = dt;
  scene->time = time;
  *(rocket_t *)scene->object = r;

  while (right - left > eps) {
    scene->dt = dt;
    scene->time = time;
    *(rocket_t *)scene->object = r;

    if (f_m1 < f_m2) {
      right = m2;
      m2 = m1;
      m1 = right - (right - left) / phi;

      f_m2 = f_m1;
      f_m1 = velocity_at_landing(scene, m1);

    } else {
      left = m1;
      m1 = m2;
      m2 = left + (right - left) / phi;

      f_m1 = f_m2;
      f_m2 = velocity_at_landing(scene, m2);
    }
  }

  scene->dt = dt;
  scene->time = time;
  *(rocket_t *)scene->object = r;

  return (left + right) / 2;
}

/// @brief Simulate landing with a hoverslam.
/// The time to ignite is found by the golden_search_hoverslam function
/// @param eps Precision for the search algorithm
/// @param print Print data during flight?
/// @param log Log simulation data to a file?
/// @return The struct of time to start the burn, rocket stats after land and
/// number of iterations during simulation
result_t hoverslam_simulation(simulator_t *scene, double eps, bool print, bool log) {
  logger_t l = (logger_t){NULL, NULL};
  if (log) {
    l = logger_init("hoverslam_sim.csv");
    assert(l.file);
    fprintln(l.file, ROCKET_LOG_HEADER);
  }

  double time_to_burn = golden_search_hoverslam(scene, eps);

  int it = 0;
  event_type_t event = EV_NONE;
  rocket_t *r = (rocket_t *)scene->object;
  rocket_t prev;

  while (event != EV_GROUND_CONTACT) {
    it++;
    prev = *r;

    if (r->time >= time_to_burn && r->thrust_percent == 0 && r->fuel_mass > 0)
      CHANGE_THRUST(*r, 1);

    scene->take_step(scene);
    event = scene->event_detector(scene, &prev);

    if (log && is_almost_integer(r->time, 0.01))
      logger_write_rocket(&l, r);
    if (print)
      PRINT_ROCKET(*r);

    if (event == EV_UNSTABLE || r->coords.z <= 0)
      break;
  }

  scene->event_interpolator(scene, &prev, event);

  if (l.file)
    logger_free(&l);

  return (result_t){*r, time_to_burn, it};
}

void usage() {
  puts("OPTIONS:\n"
       "--print\t\t\tPrint simulation\n"
       "--log\t\t\tLog simulation into cvs file\n"
       "--rocket <file>\t\tSpecify file with simulation parameters\n"
       "--dt <number>\t\tChange dt variable(default is 2e-3)\n"
       "--eps <number>\tChange eps variable(default is 1e-4)\n"
       "-h\t\t\tPrint this help message");
}

int main(int argc, char *argv[]) {
  double dt = 0.002, eps = 1e-4;
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
    } else if (strcmp(token, "--eps") == 0) {
      if (!argv[i + 1]) {
        fprintln(stderr, "Expected value after '%s'!", token);
        return -1;
      }
      if ((eps = atof(argv[++i])) == 0.0) {
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

  result_t result = hoverslam_simulation(&scene, eps, to_print, to_log);

  result.r.d.self = &result.r;
  println("Rocket stats after land:\n{}\nTime to start hoverslam:%f\nTotal "
          "iterations during simulation:%d",
          &result.r, result.time_to_burn, result.it);

  rocket_free(r);

  return 0;
}
