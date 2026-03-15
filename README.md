# Autonomous Robot Navigation Simulator

A 2D robotics simulation environment built from scratch in C++17. The project implements a custom physics engine, differential-drive robot model, PID-based path following, simulated LIDAR, log-odds occupancy grid mapping, RRT path planning, and path shortcutting. The user sets a goal by left-clicking in the world; the system plans a collision-free path from the robot’s current pose to the goal, shortens it via collision-safe shortcuts, and the robot executes the path using heading and speed controllers while continuously updating the map from LIDAR.

---

## Architecture

**Math and rendering.** A small `Vec2` library provides vector arithmetic, dot/cross products, normalization, rotation, and distance. Rendering is handled by an SFML-based wrapper with fixed timestep (60 Hz), drawing primitives (circles, lines, filled rectangles) for bodies, LIDAR rays, occupancy grid, RRT tree, and path.

**Physics.** Rigid bodies have position, velocity, angle, angular velocity, and inverse mass/inertia. The `World` steps the simulation with Euler integration, maintains a spatial hash grid for broad-phase collision detection, and resolves circle–circle and circle–AABB contacts with impulses, friction, and positional correction. The robot is a circular rigid body driven by differential-drive kinematics (wheel speeds converted to linear and angular velocity).

**Robot and control.** The `Robot` wraps a world body and exposes `setWheelSpeeds` and `updateKinematics`. The `RobotController` uses two PIDs (heading and speed) to drive toward the current waypoint; when the path is a list of waypoints from the planner, the robot follows it. Waypoint lists are produced by RRT and then shortened by the path smoother.

**Sensing and mapping.** A `Lidar` sensor casts rays from the robot’s pose against walls (axis-aligned boxes) and circular bodies, with optional Gaussian noise. Ranges update an `OccupancyGrid` (log-odds, world-to-grid conversion). The grid is queried for occupancy when planning and smoothing paths.

**Planning.** RRT grows a tree from the robot’s position with random samples in world space and a goal bias. Edges are checked for collision against the occupancy grid. When a node lies within the goal threshold, the path is extracted by backtracking parents. `PathSmoother::shortcut` shortens the polyline by replacing subpaths with straight segments when the segment is collision-free in the grid. The resulting waypoints are sent to the robot controller.

---

## Features

- **Vector math:** `Vec2` with arithmetic, dot/cross, magnitude, normalize, rotate, perpendicular, distance.
- **Rendering:** SFML window, fixed timestep loop, draw circle/line/filled rect.
- **Rigid-body physics:** Forces, torques, Euler integration, AABB, spatial hash, circle–circle and circle–AABB collision, impulse resolution with restitution and friction, positional correction.
- **Differential-drive robot:** Wheel base, wheel speeds, kinematics updating body velocity and angular velocity.
- **PID control:** Generic PID with `update(error, dt)` and output limits; heading and speed controllers; waypoint following with tolerance and stop on empty list.
- **LIDAR:** Configurable range and beam count; ray–segment and ray–circle intersection; noise; visualization of rays.
- **Occupancy grid:** Log-odds updates from LIDAR rays (free along ray, occupied at hit); world–grid conversion; visualization of occupied cells.
- **RRT:** Node tree, nearest-node search, steer with step size, occupancy-grid collision check, goal bias, path extraction, tree and path visualization.
- **Path smoothing:** Shortcutting of RRT path with collision checks against the grid before execution.
- **Click-to-goal:** Left-click sets goal; RRT and controller reset; path planned and shortcut; robot follows; goal drawn on screen.

---

## Building and Running

**Prerequisites:** CMake 3.15+, C++17 compiler, SFML 3 (Graphics, Window, System). On macOS with Homebrew: `brew install sfml`.

**Build:**

```bash
mkdir -p build && cd build
cmake ..
make
```

**Run:** From `build`, run `./simulator`. Use the mouse to left-click a goal in the window. The robot will plan an RRT path, shorten it, and drive to the goal while updating the occupancy grid from LIDAR. The RRT tree, final path, walls, and occupied grid cells are drawn each frame.

---

## Project Structure

- **`CMakeLists.txt`** — Build configuration and source list.
- **`include/`** — Headers: `math/` (Vec2, Ray), `physics/` (RigidBody, World, Collision, Resolver, AABB, SpatialHashGrid, Box, Robot), `rendering/Renderer.h`, `control/PID.h`, `robot/RobotController.h`, `sensors/Lidar.h`, `mapping/OccupancyGrid.h`, `planning/Node.h`, `planning/RRT.h`, `planning/PathSmoother.h`.
- **`src/`** — Implementations: `main.cpp`, `math/Ray.cpp`, `physics/*.cpp`, `sensors/Lidar.cpp`, `planning/RRT.cpp`, `planning/PathSmoother.cpp`.
- **`tests/`** — `vec2_test.cpp` for vector math.

---

## Possible Extensions

Cubic spline or B-spline path smoothing would yield smoother trajectories. Automatic replanning (e.g., when the robot reaches the goal or when the map changes) would support multi-goal or dynamic environments. Re-enabling or tuning wall collision response in the physics step would keep the robot from passing through boundaries when desired.
