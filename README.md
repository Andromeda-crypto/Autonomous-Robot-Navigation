# Autonomous Robot Navigation Simulator
## A First-Principles Robotics Stack in C++17

![C++](https://img.shields.io/badge/C++-17-blue.svg) ![SFML](https://img.shields.io/badge/Graphics-SFML-green.svg) ![Robotics](https://img.shields.io/badge/Focus-Robotics%20&%20Physics-orange.svg)

A high-performance 2D robotics simulation environment built from scratch. This project demonstrates a complete autonomy stack—including a custom physics engine, differential-drive kinematics, PID control, LIDAR-based sensing, Log-Odds occupancy mapping, and RRT-based path planning.

---

## Tech Stack

*   **Language:** C++17 (Modern C++ standards)
*   **Build System:** CMake 3.15+
*   **Graphics & Windowing:** SFML 3 (Graphics, Window, System)
*   **Physics:** Custom Rigid Body Euler integration with impulse resolution.
*   **Algorithms:** RRT (Rapidly-exploring Random Tree), Path Smoothing, Log-Odds Probabilistic Mapping, PID Controllers.

---

## Engineering Highlights

### 1. Perception & Mapping (LIDAR)
*   **Simulated Sensors**: Implemented a 360° LIDAR sensor using ray-casting against complex geometry (AABBs and circles) with configurable Gaussian noise profiles.
*   **Probabilistic Occupancy Grid**: Built a log-odds mapping system that updates cell states based on sensor data, effectively handling uncertainty and noise in real-time.

### 2. Path Planning & Navigation
*   **Motion Planning**: Implemented RRT to navigate non-convex obstacle spaces, featuring a goal-biased sampling strategy.
*   **Path Smoothing**: Developed a "shortcutting" algorithm that optimizes raw RRT polylines into direct, collision-free waypoint lists using line-of-sight checks against the occupancy grid.

### 3. Physics & Robot Kinematics
*   **Custom Physics Engine**: Built a rigid-body simulator from first principles, featuring impulse-based collision resolution (restitution/friction) and spatial partitioning (Spatial Hash Grid) for efficient broad-phase detection.
*   **Differential Drive**: Modeled non-holonomic kinematics for two-wheeled robots, converting wheel velocities to global linear/angular motion.

### 4. Control Systems
*   **Dual-Loop PID Control**: Implemented generic PID controllers for precise heading and velocity tracking, enabling smooth waypoint transition and arrival.

---

## Project Structure

*   **`include/math/`**: Custom `Vec2` library with vector calculus (Dot/Cross products, normalization).
*   **`include/physics/`**: Rigid bodies, Collision Resolvers (Circle-Circle, Circle-AABB), and Spatial Hash Grids.
*   **`include/mapping/`**: Log-Odds Occupancy Grid data structures and probability conversion logic.
*   **`include/planning/`**: RRT implementation and Path Smoother heuristics.
*   **`include/robot/`**: High-level `RobotController` logic and kinematics models.
*   **`src/main.cpp`**: Entry point managing the 60Hz fixed-timestep simulation loop and SFML rendering.
*   **`tests/`**: Unit tests for core mathematical primitives.

---

##  Building and Running

### Prerequisites
*   **C++17** compatible compiler
*   **SFML 3.x** (e.g., `brew install sfml` on macOS)

### Build Instructions
```bash
mkdir -p build && cd build
cmake ..
make
```

### Usage
From the `build` directory, run `./simulator`.
1.  **Set Goal**: Left-click in the simulation window.
2.  **Observe**: Watch the RRT tree expand, smooth the path, and the robot execute navigation while updating its local occupancy map in real-time.

---

## Future Improvements

*   **Global SLAM**: Transition from a local occupancy grid to a global pose-graph SLAM integration.
*   **Advanced Smoothing**: Implement Cubic B-Splines or Quintic Hermite Splines for smoother velocity profiles.
*   **Dynamic Obstacles**: Enhance the physics engine to support moving bodies and implement velocity-space obstacles (VO/RVO) for avoidance.
*   **DWA Controller**: Implement a Dynamic Window Approach for better local trajectory planning in dense environments.
