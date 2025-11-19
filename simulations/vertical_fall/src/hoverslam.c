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
double velocity_at_landing(rocket_t r, double ignition_time) {
  while (true) {
    rocket_t prev_state = r;

    if (r.time >= ignition_time && r.thrust_percent == 0)
      CHANGE_THRUST(r, 1);

    UPDATE_ROCKET_STATUS(&r);

    if (r.event_detector && r.event_detector(&r, &prev_state)) {
      if (r.event_interpolator) {
        r.event_interpolator(&r, &prev_state);
      }
      break;
    }

    // Safety break if detector is not set
    if (!r.event_detector && r.coords.z <= 0)
      break;
  }

  if (isinf(r.velocity.z))
    return INFINITY;

  return fabs(r.velocity.z);
}

/// @brief Find the best time to ignite the engine using a golden-section
/// search. Treats velocity_at_landing as a function to be minimized
/// @param eps Precision
/// @return Time to start the engine
double golden_search_hoverslam(rocket_t r, double eps) {
  double g = calculate_g(r), phi = (1 + sqrt(5)) / 2; // φ ≈ 1.618
  double right = sqrt((r.coords.z * 2) / g), left = 0;
  double m1 = right - (right - left) / phi, m2 = left + (right - left) / phi;
  double f_m1 = velocity_at_landing(r, m1), f_m2 = velocity_at_landing(r, m2);

  while (right - left > eps) {
    if (f_m1 < f_m2) {
      right = m2;
      m2 = m1;
      m1 = right - (right - left) / phi;

      f_m2 = f_m1;
      f_m1 = velocity_at_landing(r, m1);
    } else {
      left = m1;
      m1 = m2;
      m2 = left + (right - left) / phi;

      f_m1 = f_m2;
      f_m2 = velocity_at_landing(r, m2);
    }
  }

  return (left + right) / 2;
}

/// @brief Simulate landing with a hoverslam.
/// The time to ignite is found by the golden_search_hoverslam function
/// @param eps Precision for the search algorithm
/// @param print Print data during flight?
/// @param log Log simulation data to a file?
/// @return The struct of time to start the burn, rocket stats after land and
/// number of iterations during simulation
result_t hoverslam_simulation(rocket_t r, double eps, bool print, bool log) {
  logger_t l = (logger_t){NULL, NULL};
  if (log) {
    l = logger_init("hoverslam_sim.csv");
    assert(l.file);
    fprintln(l.file, ROCKET_LOG_HEADER);
  }

  double time_to_burn = golden_search_hoverslam(r, eps);

  int it = 0;
  while (true) {
    it++;
    rocket_t prev_state = r;

    if (r.time >= time_to_burn && r.thrust_percent == 0 && r.fuel_mass > 0)
      CHANGE_THRUST(r, 1);

    UPDATE_ROCKET_STATUS(&r);

    if (log && is_almost_integer(r.time, 0.01))
      logger_write_rocket(&l, &r);
    if (print)
      PRINT_ROCKET(r);

    if (r.event_detector && r.event_detector(&r, &prev_state)) {
      if (r.event_interpolator) {
        r.event_interpolator(&r, &prev_state);
      }
      break;
    }

    // Safety break if detector is not set
    if (!r.event_detector && r.coords.z <= 0)
      break;
  }

  if (l.file)
    logger_free(&l);

  return (result_t){r, time_to_burn, it};
}

int main(int argc, char *argv[]) {
  double dt = 0.002, eps = 1e-4;
  bool to_print = false, to_log = false;
  double fuel_mass = 0.0, dry_mass = 0.0, altitude = 0.0;
  engine_t eng = {0};
  planet_t pl = {0};
  char *rocket_file = "rocket.dat";

  // Parsing cmd args
  for (int i = 1; i < argc; i++) {
    char *token = argv[i];
    if (strcmp(token, "--dt") == 0) {
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

  pl.mass = fparser_get_var(&fp, "planet", "mass").value;
  pl.radius = fparser_get_var(&fp, "planet", "radius").value;

  eng.thrust = fparser_get_var(&fp, "engine", "thrust").value;
  eng.consumption = fparser_get_var(&fp, "engine", "consumption").value;

  fuel_mass = fparser_get_var(&fp, "rocket", "fuel_mass").value;
  dry_mass = fparser_get_var(&fp, "rocket", "dry_mass").value;
  altitude = fparser_get_var(&fp, "rocket", "altitude").value;

  rocket_t r = start_falling(dt, dry_mass, fuel_mass, altitude, eng, pl);
  r.d.self = &r;
  if (!is_enough_deltav(&r)) {
    println("Available delta-v: %.2f\nNot enough for landing!", deltav(&r));
    return -1;
  }

  result_t result = hoverslam_simulation(r, eps, to_print, to_log);

  result.r.d.self = &result.r;
  println("Rocket stats after land:\n{}\nTime to start hoverslam:%f\nTotal "
          "iterations during simulation:%d",
          &result.r, result.time_to_burn, result.it);

  return 0;
}
