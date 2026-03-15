#include <SFML/Graphics.hpp>
#include <optional>
#include <vector>
#include <iostream>

#include "rendering/Renderer.h"
#include "math/Vec2.h"

#include "physics/World.h"
#include "physics/RigidBody.h"
#include "physics/Collision.h"
#include "physics/Box.h"
#include "physics/Robot.h"

#include "robot/RobotController.h"
#include "sensors/Lidar.h"
#include "mapping/OccupancyGrid.h"

#include "planning/RRT.h"
#include "planning/PathSmoother.h"

int main()
{
    sf::RenderWindow window(
        sf::VideoMode({800u, 600u}),
        "Robot Simulator"
    );

    window.setFramerateLimit(60);

    Renderer renderer(window);
    World world;

    // -------------------------
    // Robot Setup
    // -------------------------
    const double robotMass = 5.0;
    RigidBody robotBody(Vec2(200, 500), robotMass);
    robotBody.radius = 25.0;

    double inertia = 0.5 * robotMass * robotBody.radius * robotBody.radius;
    if (inertia > 0.0)
        robotBody.inverseInertia = 1.0 / inertia;

    RigidBody& worldRobotBody = world.addBody(robotBody);

    Robot robot(worldRobotBody);
    RobotController controller(robot);

    // -------------------------
    // Lidar + Mapping
    // -------------------------
    const double lidarRange = 300.0;
    Lidar lidar(&worldRobotBody.position, &worldRobotBody.angle, lidarRange, 36);

    const double cellSize = 10.0;
    OccupancyGrid grid(
        static_cast<int>(800 / cellSize),
        static_cast<int>(600 / cellSize),
        cellSize,
        lidarRange
    );

    // -------------------------
    // Planning
    // -------------------------
    RRT rrt;

    Vec2 goal(700.0, 100.0);
    std::vector<Vec2> path;

    bool goalSet = false;
    bool pathComputed = false;

    // -------------------------
    // Environment Walls
    // -------------------------
    Box floor(Vec2(400, 590), Vec2(400, 10));
    Box ceiling(Vec2(400, 10), Vec2(400, 10));
    Box leftWall(Vec2(10, 300), Vec2(10, 300));
    Box rightWall(Vec2(790, 300), Vec2(10, 300));

    std::vector<Box> walls = { floor, ceiling, leftWall, rightWall };

    // -------------------------
    // Fixed timestep
    // -------------------------
    sf::Clock clock;
    const double dt = 1.0 / 60.0;
    double accumulator = 0.0;

    while (window.isOpen())
    {
        double frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        // -------------------------
        // Events
        // -------------------------
        while (auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }

            if (event->is<sf::Event::MouseButtonPressed>())
            {
                auto* mouse = event->getIf<sf::Event::MouseButtonPressed>();

                if (mouse && mouse->button == sf::Mouse::Button::Left)
                {
                    goal = Vec2(
                        static_cast<double>(mouse->position.x),
                        static_cast<double>(mouse->position.y)
                    );

                    std::cout << "New goal: " << goal.x << ", " << goal.y << '\n';

                    // Reset planner and controller for new goal
                    rrt.reset(worldRobotBody.position);
                    path.clear();
                    pathComputed = false;
                    goalSet = true;

                    controller.setWaypoints({});
                    robot.setWheelSpeeds(0.0, 0.0);
                    worldRobotBody.velocity = Vec2(0.0, 0.0);
                    worldRobotBody.angularVelocity = 0.0;
                }
            }
        }

        // -------------------------
        // Physics / Planning Update
        // -------------------------
        while (accumulator >= dt)
        {
            controller.update(dt);

            lidar.update(walls, world.getBodies());

            const auto& distances = lidar.getDistances();
            for (int i = 0; i < static_cast<int>(distances.size()); ++i)
            {
                Ray ray = lidar.getBeamRay(i);
                grid.updateRay(ray.origin, ray.direction, distances[i]);
            }

            world.step(dt);

            if (goalSet && !pathComputed)
            {
                for (int i = 0; i < 200; ++i)
                {
                    if (rrt.expand(goal, grid, path))
                    {
                        std::cout << "PATH FOUND\n";

                        path = PathSmoother::shortcut(path, grid);
                        std::cout << "Waypoints: " << path.size() << '\n';

                        controller.setWaypoints(path);
                        pathComputed = true;
                        break;
                    }
                }
            }

            accumulator -= dt;
        }

        // -------------------------
        // Rendering
        // -------------------------
        renderer.clear();

        // Draw goal only when active
        if (goalSet)
            renderer.drawCircle(goal, 6);

        // Draw bodies
        for (const auto& body : world.getBodies())
            renderer.drawCircle(body.position, body.radius);

        // Draw occupancy grid
        for (int y = 0; y < grid.getHeight(); ++y)
        {
            for (int x = 0; x < grid.getWidth(); ++x)
            {
                double p = grid.getProbability(x, y);

                if (p > 0.7)
                {
                    sf::Color c(200, 50, 50, 180);
                    Vec2 min(x * cellSize, y * cellSize);
                    Vec2 max((x + 1) * cellSize, (y + 1) * cellSize);
                    renderer.drawFilledRect(min, max, c);
                }
            }
        }

        // Draw lidar
        // Draw lidar
        const auto& distances = lidar.getDistances();

        for (int i = 0; i < static_cast<int>(distances.size()); ++i)
        {
            Ray ray = lidar.getBeamRay(i);
            Vec2 end = ray.origin + ray.direction * distances[i];
            renderer.drawLine(ray.origin, end);
        }

        // Draw RRT tree
        for (const auto& node : rrt.getTree())
        {
            if (node.parent != -1)
            {
                const Vec2& parent = rrt.getTree()[node.parent].position;
                renderer.drawLine(parent, node.position);
            }
        }

        // Draw final path
        for (size_t i = 1; i < path.size(); ++i)
            renderer.drawLine(path[i - 1], path[i]);

        // Draw walls
        for (const auto& wall : walls)
        {
            Vec2 min = wall.center - wall.halfSize;
            Vec2 max = wall.center + wall.halfSize;

            renderer.drawLine(Vec2(min.x, min.y), Vec2(max.x, min.y));
            renderer.drawLine(Vec2(max.x, min.y), Vec2(max.x, max.y));
            renderer.drawLine(Vec2(max.x, max.y), Vec2(min.x, max.y));
            renderer.drawLine(Vec2(min.x, max.y), Vec2(min.x, min.y));
        }

        renderer.display();
    }

    return 0;
}