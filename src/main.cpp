#include <SFML/Graphics.hpp>
#include <optional>

#include "rendering/Renderer.h"
#include "math/Vec2.h"
#include "physics/World.h"
#include "physics/RigidBody.h"

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

    // Create a body
    RigidBody body(Vec2(400,300), 1.0);

    body.applyForce(Vec2(5000.0, 0.0));

    world.addBody(body);

    // Fixed timestep variables
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
            accumulator -= dt;
        }
        // Rendering
        renderer.clear();

    for (const auto& body : world.getBodies())
        {
        renderer.drawCircle(body.position, 20);
        }

    renderer.display();
    }
}