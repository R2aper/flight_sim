#ifndef EVENTS_H
#define EVENTS_H

/**
 * @file events.h
 * @brief Defines event types for simulation event detection
 */

/**
 * @enum event_type_t
 * @brief Enumerates the types of simulation events that can be detected
 *
 * This enum is used by the event detector function to classify the outcome of
 * a simulation step
 */
typedef enum {
  /// No event has occurred
  EV_NONE,

  /// The rocket has made contact with the ground
  EV_GROUND_CONTACT,

  /// The simulation has become unstable
  EV_UNSTABLE,

  /// The rocket has run out of fuel
  EV_OUT_OF_FUEL,

  /// A custom event defined by the user simulation
  EV_CUSTOM

} event_type_t;

#endif // EVENTS_H
