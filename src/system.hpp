#pragma once

#include <vector>

#include "./particle.hpp"
#include "../raylib/include/raymath.h"

class Particle_system
{
    std::vector<Particle> particles;
    Vector2 gravity{0.0f, 900.0f};
    float dt = 1.0f / 60.0f;

public:
    int boundary_radius{250};
    Vector2 boundary_center{};
    Particle_system() = default;
    Particle_system(Particle_system const &other) = default;
    Particle_system &operator=(Particle_system const &other) = default;
    Particle_system(Particle_system &&other) = default;
    Particle_system &operator=(Particle_system &&other) = default;

    Particle& add_particle(const Particle &particle);

    void update();

    std::vector<Particle> const &get_particles() const;

private:
    void apply_gravity();
    void apply_boundary()
    {
        for (auto & particle : particles)
        {
            Vector2 radius = Vector2Subtract(boundary_center, particle.position);
            float sq_dis = radius.x * radius.x + radius.y * radius.y;

            if (sq_dis > ((boundary_radius * boundary_radius) - particle.radius))
            {
                Vector2 tangent = Vector2Negate(Vector2Normalize(radius));

            }

        }


    }
    void update_particles();
};
