cmake_minimum_required(VERSION 3.16)

project(AutonomousRobotNavigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include_directories(include)


add_executable(vec2_test
    tests/vec2_test.cpp
)


target_compile_options(vec2_test PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)