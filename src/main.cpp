#include <SFML/Graphics.hpp>
#include <optional>

#include "rendering/Renderer.h"
#include "math/Vec2.h"

#include "physics/World.h"
#include "physics/RigidBody.h"
#include "physics/Collision.h"
#include "physics/Box.h"

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

    // Dynamic ball
    RigidBody ball(Vec2(400,100), 1.0);
    ball.radius = 20;

    world.addBody(ball);

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

        // Draw circle
        for (const auto& body : world.getBodies())
        {
            renderer.drawCircle(body.position, body.radius);
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