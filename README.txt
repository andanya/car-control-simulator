# Self-Driving Car Simulation

This repository contains an autonomous driving algorithm for a simulated multi-lane road environment.

The controller is implemented in the `solve` method and is designed to drive the car to the end of the road while:

* avoiding collisions with obstacles;
* yielding appropriately to pedestrians;
* navigating around crosswalks and other road features;
* maintaining reasonably smooth and human-like driving behavior.

The goal is not merely to reach the end of the road, but to do so in a way that would be acceptable for both passengers and pedestrians if the same driving behavior were used by a real autonomous vehicle.

## Algorithm

At each simulation step, the controller receives information about:

* the current state of the car;
* detected pedestrians;
* obstacles;
* crosswalks;
* the geometry of the road.

Based on this information, it selects the car's acceleration and steering angle and advances the simulation.

The implementation prioritizes safe obstacle avoidance and pedestrian behavior while attempting to keep acceleration, braking, and steering reasonably smooth.

## Simulation Environment

The simulator is implemented in C++ using Qt.

The main environment interface is provided by the `World` class.

### `World`

The following constants describe the road:

```cpp
static constexpr int ROAD_LENGTH = 10000; // road length
static constexpr int LANES_NUMBER = 4;    // number of lanes
static constexpr int LANE_WIDTH = 30;     // lane width
```

The controller can use the following methods:

```cpp
Car getMyCar() const;
```

Returns the current state of the controlled car.

```cpp
double getRoadWidth() const;
```

Returns the total width of the road.

```cpp
void makeStep(double a, double df);
```

Advances the simulation by commanding the car to:

* accelerate with acceleration `a`;
* change the steering angle by `df` radians.

```cpp
vector<DetectedPedestrian> getPedestrians() const;
```

Returns the currently detected pedestrians.

```cpp
vector<Obstacle> getObstacles() const;
```

Returns the detected obstacles.

```cpp
vector<Crosswalk> getCrosswalks() const;
```

Returns the crosswalks in the simulation.

### `Car`

The `Car` class defines the dimensions of the vehicle:

```cpp
static constexpr double LENGTH = 40;
static constexpr double WIDTH = 18;
```

## Building

The project can be compiled using the Qt framework.

Open the project in Qt Creator and build it using the standard Qt build process.

## Alternative Implementations

The driving algorithm itself is not fundamentally tied to C++. It can be reimplemented in another language, provided that the functionality exposed by `world.h` is reproduced or replaced by a simulator following equivalent principles.
