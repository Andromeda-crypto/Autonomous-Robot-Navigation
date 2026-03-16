cmake_minimum_required(VERSION 3.15)
project(RobotSimulator)

set(CMAKE_CXX_STANDARD 17)

find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

include_directories(include)

add_executable(simulator
    src/main.cpp
    src/math/Ray.cpp
    src/physics/RigidBody.cpp
    src/physics/Robot.cpp
    src/physics/Collision.cpp
    src/physics/Resolver.cpp
    src/physics/World.cpp
    src/physics/SpatialHashGrid.cpp
    src/sensors/Lidar.cpp
    src/planning/RRT.cpp
    src/planning/PathSmoother.cpp
    src/planning/TrajectoryGenerator.cpp
)

target_link_libraries(simulator
    SFML::Graphics
    SFML::Window
    SFML::System
)