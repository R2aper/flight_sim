# Rocket Simulation Library (`librocket`)

This library provides a collection of modules for building rocket flight simulations. It includes components for physics modeling, PID control, data logging, and configuration parsing.

## Modules

The library is composed of the following modules:

-   **`rocket`**: Defines the core data structures for the rocket (`rocket_t`), its engine (`engine_t`), and the planetary environment (`planet_t`). It handles the physics and state updates for the simulation.
-   **`PID`**: A simple Proportional-Integral-Derivative (PID) controller implementation that can be used for guidance and control systems (e.g., controlling thrust for a soft landing).
-   **`logger`**: A buffered file logger (`logger_t`) for efficiently recording simulation data, such as the rocket's state over time.
-   **`fparser`**: A file parser (`fparser_t`) for reading simulation parameters from configuration files. It supports simple `key = value` pairs grouped into sections.
-   **`utils`**: A set of utility functions and constants, including vector math (`vec3_t`), physical constants, and helper functions.
-   **`display`**: Provides a generic interface for displaying the state of different data structures in the simulation.

## Getting Started

The primary header to include in your project is `rocketlib.h`, which provides access to all the library's functionality.

```c
#include "rocketlib.h"
```

## Building

The library is built as a shared object (`librocket.so`) using the [Meson build system](https://mesonbuild.com/).

To build the library, navigate to the `lib` directory and run:

```bash
meson setup build
ninja -C build
```

This will compile the source files located in `lib/src` and create the shared library in the `lib/build` directory. The compiled library can then be linked against your simulation applications.