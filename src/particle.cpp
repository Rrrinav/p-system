#include "./particle.hpp"
#include "../raylib/include/raymath.h"

void Particle::update(float dt)
{
    Vector2 displacement = Vector2Subtract(position, position_last); // Get velocity approximation
    Vector2 new_position = Vector2Add(Vector2Add(position, displacement), Vector2Scale(acceleration , (dt * dt)));

    position_last = position;  // Save current position before updating
    position = new_position;   // Apply new position
    acceleration = {};         // Reset acceleration
}

void Particle::accelerate(const Vector2 &force)
{
    acceleration = Vector2Add(acceleration, force);
}

void Particle::set_velocity(const Vector2 &v, float dt)
{
    position_last = Vector2Subtract(position, Vector2Scale(v, dt));
}

void Particle::add_velocity(const Vector2 &v, float dt)
{
    position_last = Vector2Subtract(position_last, Vector2Scale(v, dt));
}

Vector2 Particle::get_velocity() const
{
    return Vector2Subtract(position, position_last);
}
