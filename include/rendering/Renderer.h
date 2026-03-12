#pragma once

#include <SFML/Graphics.hpp>
#include "math/Vec2.h"

class Renderer
{
private:
    sf::RenderWindow& window;

public:

    Renderer(sf::RenderWindow& window)
        : window(window) {}

    void clear()
    {
        window.clear(sf::Color::Black);
    }

    void display()
    {
        window.display();
    }

    void drawCircle(const Vec2& position, float radius)
    {
        sf::CircleShape circle(radius);

        circle.setFillColor(sf::Color::Green);
        circle.setPosition({
        static_cast<float>(position.x - radius),
        static_cast<float>(position.y - radius)
});

        window.draw(circle);
    }

    void drawLine(const Vec2& a, const Vec2& b)
    {
        sf::Vertex line[2];

        line[0].position = {
            static_cast<float>(a.x),
            static_cast<float>(a.y)
        };
        line[0].color = sf::Color::White;
        line[1].position = {
            static_cast<float>(b.x),
            static_cast<float>(b.y)
        };
        line[1].color = sf::Color::White;

        window.draw(line, 2, sf::PrimitiveType::Lines);
    }

    void drawFilledRect(const Vec2& min, const Vec2& max, const sf::Color& color)
    {
        sf::RectangleShape rect;
        rect.setPosition({
            static_cast<float>(min.x),
            static_cast<float>(min.y)
        });
        rect.setSize({
            static_cast<float>(max.x - min.x),
            static_cast<float>(max.y - min.y)
        });
        rect.setFillColor(color);
        window.draw(rect);
    }
};