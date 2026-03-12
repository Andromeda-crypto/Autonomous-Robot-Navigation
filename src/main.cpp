#include <SFML/Graphics.hpp>
#include <optional>
#include <vector>

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

int main()
{
    sf::RenderWindow window(
        sf::VideoMode({800u, 600u}),
        "Robot Simulator"
    );

    window.setFramerateLimit(60);

    Renderer renderer(window);

    // Physics world
    World world;

    // Robot body created inside the world, robot wrapper controls that same instance
    double robotMass = 5.0;
    RigidBody robotBody(Vec2(200, 500), robotMass);
    robotBody.radius = 25.0;
    // Approximate moment of inertia for a solid disk
    {
        double inertia = 0.5 * robotMass * robotBody.radius * robotBody.radius;
        if (inertia > 0.0)
            robotBody.inverseInertia = 1.0 / inertia;
    }
    RigidBody& worldRobotBody = world.addBody(robotBody);

    Robot robot(worldRobotBody);
    RobotController controller(robot);

    // Lidar sensor mounted at robot center, uses robot heading.
    const double lidarRange = 300.0;
    Lidar lidar(&worldRobotBody.position, &worldRobotBody.angle, lidarRange, 36);

    // Occupancy grid covering the window (800x600) with 10x10 cells.
    const double cellSize = 10.0;
    OccupancyGrid grid(
        static_cast<int>(800 / cellSize),
        static_cast<int>(600 / cellSize),
        cellSize,
        lidarRange);

    // Simple waypoint list for autonomous driving
    std::vector<Vec2> waypoints = {
        Vec2(600, 500),
        Vec2(600, 100),
        Vec2(200, 100),
        Vec2(200, 500)
    };
    controller.setWaypoints(waypoints);

    // Walls
    Box floor(Vec2(400,590), Vec2(400,10));
    Box ceiling(Vec2(400,10), Vec2(400,10));
    Box leftWall(Vec2(10,300), Vec2(10,300));
    Box rightWall(Vec2(790,300), Vec2(10,300));

    std::vector<Box> walls = {floor, ceiling, leftWall, rightWall};

    // Fixed timestep
    sf::Clock clock;
    const double dt = 1.0 / 60.0;
    double accumulator = 0.0;

    while (window.isOpen())
    {
        double frameTime = clock.restart().asSeconds();
        accumulator += frameTime;

        // Event handling
        while (auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // Physics update
        while (accumulator >= dt)
        {
            // High-level controller sets wheel speeds and advances robot toward waypoints.
            controller.update(dt);
            // Update lidar after robot motion
            lidar.update(walls, world.getBodies());

            // Update occupancy grid from lidar beams
            const auto& distances = lidar.getDistances();
            for (int i = 0; i < static_cast<int>(distances.size()); ++i)
            {
                Ray ray = lidar.getBeamRay(i);
                grid.updateRay(ray.origin, ray.direction, distances[i]);
            }
            // World then integrates all bodies and handles collisions.
            world.step(dt);

            // Check circle vs wall collisions
            for (auto& body : const_cast<std::vector<RigidBody>&>(world.getBodies()))
            {
                for (const auto& wall : walls)
                {
                    Manifold m = checkCircleAABB(body, wall);

                    if (m.colliding)
                    {
                        // simple positional correction
                        body.position += m.normal * m.penetrationDepth;

                        // bounce velocity
                        double vn = body.velocity.dot(m.normal);

                        if (vn < 0)
                        {
                            body.velocity -= m.normal * (1.8 * vn);
                        }
                    }
                }
            }

            accumulator -= dt;
        }

        // Rendering
        renderer.clear();

        // Draw bodies (circles)
        for (const auto& body : world.getBodies())
        {
            renderer.drawCircle(body.position, body.radius);
        }

        // Visualize occupancy grid as an overlay
        for (int y = 0; y < grid.getHeight(); ++y)
        {
            for (int x = 0; x < grid.getWidth(); ++x)
            {
                double p = grid.getProbability(x, y);
                if (p < 0.3 && p > 0.0)
                {
                    // Likely free space: light blue
                    sf::Color c(50, 50, 150, 100);
                    Vec2 min(x * cellSize, y * cellSize);
                    Vec2 max((x + 1) * cellSize, (y + 1) * cellSize);
                    renderer.drawFilledRect(min, max, c);
                }
                else if (p > 0.7)
                {
                    // Likely occupied: red
                    sf::Color c(200, 50, 50, 180);
                    Vec2 min(x * cellSize, y * cellSize);
                    Vec2 max((x + 1) * cellSize, (y + 1) * cellSize);
                    renderer.drawFilledRect(min, max, c);
                }
            }
        }

        // Visualize lidar rays
        const auto& distances = lidar.getDistances();
        for (int i = 0; i < static_cast<int>(distances.size()); ++i)
        {
            Ray ray = lidar.getBeamRay(i);
            Vec2 end = ray.origin + ray.direction * distances[i];
            renderer.drawLine(ray.origin, end);
        }

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
}