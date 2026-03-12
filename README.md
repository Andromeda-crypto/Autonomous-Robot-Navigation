## Autonomous Robot Navigation Simulator

Autonomous-Robot-Navigation is a small 2D robotics simulator built from scratch in modern C++.  
It combines a custom physics engine, an SFML renderer, a differential-drive robot model, PID-based control,  
LIDAR-style range sensing, and an occupancy grid mapping pipeline. Planned next stages add RRT path planning,  
path smoothing, and full goal-driven navigation.

---

## Features Implemented So Far

### Math & Core Infrastructure (Parts 1–2)

- **Custom vector math (`Vec2`)**
  - Basic arithmetic, dot/cross products, magnitude/normalize, rotation, perpendicular, distance.
- **Rendering & windowing**
  - SFML-based `Renderer` abstraction.
  - Fixed timestep simulation loop (60 Hz).
  - Simple drawing helpers: circles, lines.

### Physics Engine (Parts 3–6)

- **Rigid body model**
  - Position, velocity, forces, angle, angular velocity, torque.
  - Inverse mass and inverse inertia for dynamic vs static bodies.
  - Circular bodies with configurable radius and friction coefficients.
- **Integration and world management**
  - `World` class for storing bodies and stepping the simulation.
  - Euler integration for linear and angular motion.
- **Collision detection**
  - Axis-aligned bounding boxes (AABB) per body.
  - Spatial hash grid broad-phase (reduces \(O(N^2)\) pair checks).
  - Narrow-phase circle–circle and circle–AABB collision tests.
  - `Manifold` structure carrying normal, penetration depth, etc.
- **Collision resolution**
  - Impulse-based resolution with restitution.
  - Angular impulses.
  - Static and dynamic friction.
  - Positional correction to prevent interpenetration.
- **Stable stacking & static bodies**
  - Support for immovable bodies (infinite mass).
  - World boundary walls.
  - Iterative solver loop for more stable stacks.

### Robot Body & Low-Level Control (Parts 7–8)

- **Differential-drive robot wrapper**
  - `Robot` struct wrapping a `RigidBody&` and modeling a circular robot with two wheels.
  - Wheel configuration: wheel base, wheel radius, left/right wheel speeds.
  - Implements differential drive kinematics:
    - \(v = \frac{v_r + v_l}{2}\)
    - \(\omega = \frac{v_r - v_l}{L}\)
  - Updates the underlying rigid body’s linear and angular velocities each timestep.
- **PID-based control**
  - Generic `PID` class with `update(error, dt)` and output clamping.
  - `RobotController`:
    - **Heading controller**: PID on heading error (desired vs current robot angle).
    - **Speed controller**: PID on linear speed error.
    - Converts \((v, \omega)\) commands into left/right wheel speeds.
  - **Waypoint following**
    - Robot follows a list of waypoints that form a rectangular track inside the world.
    - Automatic advance to the next waypoint when within a small tolerance.

### LIDAR & Ray Casting (Part 9)

- **Ray utilities**
  - `Ray` and `RayHit` types.
  - Ray vs line-segment intersection helper.
  - Ray vs circle intersection for detecting hits on circular bodies.
- **LIDAR sensor**
  - `Lidar` class mounted on the robot, parameterized by:
    - Pointer to robot position and heading.
    - Maximum range.
    - Number of beams.
  - Emits a fan of rays (currently 360° coverage).
  - Performs intersection tests against:
    - Static walls (boxes decomposed into 4 line segments).
    - Dynamic circular bodies.
  - Adds configurable Gaussian noise to range measurements.
- **Visualization**
  - All rays are rendered as white lines from the sensor origin to the hit point (or max range).

### Occupancy Grid Mapping (Part 10)

- **Occupancy grid data structure**
  - `OccupancyGrid` using log-odds representation.
  - Configured to cover the SFML window (e.g., 800×600) with a tunable cell size (default 10 px).
  - Provides:
    - World→grid coordinate conversion.
    - Per-cell probability query.
- **Sensor integration**
  - Each LIDAR beam is used to update the grid:
    - Cells along the beam up to just before a hit are marked more likely **free**.
    - The hit cell (if within range) is marked more likely **occupied**.
  - Log-odds are clamped to avoid saturation.
- **Grid visualization**
  - Grid is drawn as a semi-transparent overlay:
    - Free-ish cells (low occupancy probability) tinted light blue.
    - Occupied-ish cells (high occupancy probability) tinted red.
  - LIDAR rays are drawn on top so you can visually correlate sensor readings with the map.
- **End-to-end behavior**
  - As the robot drives its waypoint track:
    - LIDAR scans the environment.
    - The occupancy grid gradually “paints” the room boundaries and explored free space.

---

## What Remains (Roadmap)

### Part 11 — RRT Path Planner

- ⬜ **11.1 Node struct** – RRT node representation (position, parent index, cost).
- ⬜ **11.2 `nearestNode()`** – find nearest node in the tree to a random sample.
- ⬜ **11.3 `steer()`** – step from a node toward a sample by a fixed step size.
- ⬜ **11.4 Collision checking** – ensure edges do not cross occupied map cells or walls.
- ⬜ **11.5 RRT expansion** – main RRT loop building a tree of feasible states.
- ⬜ **11.6 Goal detection** – detect when a node is “close enough” to the goal region.
- ⬜ **11.7 Path extraction** – backtrack parents from goal node to root to get a path.
- ⬜ **11.8 Tree visualization** – draw nodes and edges of the RRT inside the world.

### Part 12 — Path Smoothing

- ⬜ **12.1 Path shortcutting** – repeatedly shortcut polyline segments that remain collision-free.
- ⬜ **12.2 Cubic spline interpolation** – fit a smooth curve (e.g., cubic B-spline) through waypoints.
- ⬜ **12.3 Sample smooth path** – generate a dense sequence of poses along the spline.
- ⬜ **12.4 Visualize curve** – render the smoothed path overlayed on the environment.

### Part 13 — Full Integration

- ⬜ **13.1 Click to set goal** – mouse interaction to place a navigation goal in the world.
- ⬜ **13.2 RRT path planning** – run RRT from robot’s current pose to the clicked goal.
- ⬜ **13.3 Path smoothing** – post-process raw RRT path via shortcutting and splines.
- ⬜ **13.4 PID path following** – track the smoothed path using the existing PID controllers.
- ⬜ **13.5 LIDAR mapping updates** – keep mapping live during navigation and planning.
- ⬜ **13.6 Automatic replanning** – trigger new RRT runs when the environment/map changes.

---

## Building and Running

### Prerequisites

- **CMake** ≥ 3.15  
- **C++17** compiler (e.g., `clang++`, `g++`)  
- **SFML 3** (Graphics, Window, System components)

On macOS with Homebrew, for example:

```bash
brew install sfml
```

### Configure and build

From the project root:

```bash
mkdir -p build
cd build
cmake ..
make
```

This produces an executable named `simulator`.

### Run

From the `build` directory:

```bash
./simulator
```

You should see:

- A green circular robot moving around the box-shaped world following a rectangular waypoint track.
- White LIDAR rays emanating from the robot.
- A colored occupancy grid overlay gradually forming a map of walls and free space.

---

## Project Structure (High-Level)

- `CMakeLists.txt` – build configuration.
- `include/`
  - `math/Vec2.h` – 2D vector math utilities.
  - `math/Ray.h` – ray and intersection structures.
  - `physics/` – rigid body, world, collision, manifolds, spatial hash, robot wrapper.
  - `rendering/Renderer.h` – thin SFML wrapper for drawing primitives.
  - `control/PID.h` – generic PID controller.
  - `robot/RobotController.h` – heading/speed PID control and waypoint following.
  - `sensors/Lidar.h` – simulated 2D LIDAR sensor.
  - `mapping/OccupancyGrid.h` – log-odds occupancy grid and ray-based updates.
- `src/`
  - `main.cpp` – brings everything together into a running simulator.
  - `math/Ray.cpp` – ray/segment intersection implementation.
  - `physics/*.cpp` – rigid body, world stepping, collision resolution, robot implementation.
  - `sensors/Lidar.cpp` – LIDAR ray casting and noise model.
- `tests/`
  - `vec2_test.cpp` – unit tests for the vector math layer.

---

## How to Extend Next

Once you are comfortable with the current simulator, the natural next steps are:

- Implement the **RRT planner** (Part 11) that uses the occupancy grid for collision checks.
- Add **path smoothing** (Part 12) for more realistic and comfortable trajectories.
- Connect everything in a **full navigation loop** (Part 13), where:
  - You click to set a goal.
  - RRT finds a collision-free path.
  - The path is smoothed and fed to the robot via the existing PID controllers.
  - LIDAR and mapping keep updating as the robot drives, with replanning when needed.

This will turn the current physics-and-sensing sandbox into a complete miniature autonomous navigation stack.