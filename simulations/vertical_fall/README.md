# Rocket Descent Optimization

## Problem Description

A rocket is at the apex of its trajectory, at a known height *h* with zero initial velocity. It will fall vertically under gravity. The rocket has a total mass *m* (which includes fuel) and an engine that provides a constant thrust *F* while consuming fuel at a constant rate.

**The goal is to land the rocket softly (with zero velocity at zero altitude) while minimizing fuel consumption.** This involves finding the optimal time to ignite the engine during the descent.

## Assumptions

-   Air resistance is neglected.
-   The descent is perfectly vertical.

## Solution Approaches

This problem is solved using two different strategies for controlling the rocket's descent.

### 1. Hoverslam (Suicide Burn)

This strategy involves a period of free fall followed by a single, continuous burn at maximum thrust. The burn is timed precisely to bring the rocket to a complete stop at ground level (h=0, v=0). This is the most fuel-efficient powered descent, as the engine runs for the minimum possible time.

The implementation finds the optimal engine ignition time using a **golden-section search** algorithm. This search method minimizes the rocket's final velocity upon landing by iteratively refining the ignition time. The objective function for this search simulates the descent for a given ignition time and returns the velocity at impact.

### 2. PID Controller

This strategy uses a Proportional-Integral-Derivative (PID) controller to continuously adjust the engine thrust to follow a predefined velocity profile. The goal is to make the rocket's velocity match a "target" velocity at any given altitude.

A common target velocity profile for landing is based on the free-fall equation: `v_target = -sqrt(2 * g * h)`. The controller applies thrust to slow the rocket down, trying to match its actual velocity to this target velocity. This approach provides a more controlled, gradual descent compared to the all-or-nothing hoverslam.

## How to Run the Simulation

### Prerequisites

-   A C compiler (like GCC or Clang)
-   The [Meson build system](https://mesonbuild.com/)
-   `librocket.so` must be built and available. Navigate to the `lib` directory and run:
    ```bash
    meson setup build
    ninja -C build
    ```

### Building and Running

1.  Compile the executables:
    ```bash
    ninja -C build
    ```

2.  Run the hoverslam/pid simulation:
    ```bash
    ./build/hoverslam --log
    ./build/pid --log
    ```
    This will run the simulation and create a `csv` file with the flight data.

3.  (Optional) Visualize the results using the provided Python script. You will need `matplotlib` and `pandas`.
    ```bash
    pip install matplotlib pandas
    python3 stats.py hoverslam_sim.csv
    python3 stats.py pid_sim.c
    ```

    This will generate a `.png` image with plots of the flight data and print a summary to the console.