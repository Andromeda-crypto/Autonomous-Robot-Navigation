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

    double robotMass = 5.0;
    RigidBody robotBody(Vec2(200, 500), robotMass);
    robotBody.radius = 25.0;

    double inertia = 0.5 * robotMass * robotBody.radius * robotBody.radius;
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
        lidarRange);

    // -------------------------
    // Planning
    // -------------------------

    RRT rrt;
    rrt.reset(worldRobotBody.position);
    Vec2 goal(700,100);
    std::vector<Vec2> path;
    bool pathComputed = false;
    if (rrt.expand(goal, grid, path))
    {
    std::cout << "PATH FOUND\n";

    controller.setWaypoints(path);
    pathComputed = true;
    }


    // -------------------------
    // Environment Walls
    // -------------------------

    Box floor(Vec2(400,590), Vec2(400,10));
    Box ceiling(Vec2(400,10), Vec2(400,10));
    Box leftWall(Vec2(10,300), Vec2(10,300));
    Box rightWall(Vec2(790,300), Vec2(10,300));

    std::vector<Box> walls = {floor, ceiling, leftWall, rightWall};

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

        while (auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // -------------------------
        // Physics Update
        // -------------------------

        while (accumulator >= dt)
        {
            controller.update(dt);

            lidar.update(walls, world.getBodies());

            const auto& distances = lidar.getDistances();

            for (int i = 0; i < distances.size(); ++i)
            {
                Ray ray = lidar.getBeamRay(i);
                grid.updateRay(ray.origin, ray.direction, distances[i]);
            }

            world.step(dt);

            // Compute RRT path once map has some information
            if (!pathComputed)
            {
                for (int i = 0; i < 200; i++)
                {
                    if (rrt.expand(goal, grid, path))
                    {
                        controller.setWaypoints(path);
                        pathComputed = true;
                        break;
                    }
                }
                std::cout << "Tree size: " << rrt.getTree().size() << "\n";
            }

            accumulator -= dt;
        }

        // -------------------------
        // Rendering
        // -------------------------

        renderer.clear();

        // Draw bodies
        for (const auto& body : world.getBodies())
            renderer.drawCircle(body.position, body.radius);

        // Draw occupancy grid
        for (int y = 0; y < grid.getHeight(); ++y)
        {
            for (int x = 0; x < grid.getWidth(); ++x)
            {
                double p = grid.getProbability(x,y);

                if (p > 0.7)
                {
                    sf::Color c(200,50,50,180);

                    Vec2 min(x * cellSize, y * cellSize);
                    Vec2 max((x+1)*cellSize,(y+1)*cellSize);

                    renderer.drawFilledRect(min,max,c);
                }
            }
        }

        // Draw Lidar
        const auto& distances = lidar.getDistances();

        for (int i = 0; i < distances.size(); ++i)
        {
            Ray ray = lidar.getBeamRay(i);

            Vec2 end = ray.origin + ray.direction * distances[i];

            renderer.drawLine(ray.origin,end);
        }

        // Draw RRT Tree
        for (const auto& node : rrt.getTree())
        {
            if (node.parent != -1)
            {
                Vec2 parent =
                    rrt.getTree()[node.parent].position;

                renderer.drawLine(parent,node.position);
            }
        }

        // Draw final path
        for (size_t i = 1; i < path.size(); i++)
            renderer.drawLine(path[i-1],path[i]);

        // Draw walls
        for (const auto& wall : walls)
        {
            Vec2 min = wall.center - wall.halfSize;
            Vec2 max = wall.center + wall.halfSize;

            renderer.drawLine(Vec2(min.x,min.y),Vec2(max.x,min.y));
            renderer.drawLine(Vec2(max.x,min.y),Vec2(max.x,max.y));
            renderer.drawLine(Vec2(max.x,max.y),Vec2(min.x,max.y));
            renderer.drawLine(Vec2(min.x,max.y),Vec2(min.x,min.y));
        }

        renderer.display();
    }
}