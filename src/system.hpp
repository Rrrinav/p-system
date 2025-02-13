#pragma once

#include <vector>

#include "./particle.hpp"

#include "../raylib/include/raymath.h"

class Particle_system
{
    std::vector<Particle> particles;
    Vector2 gravity{0.0f, 900.0f};
    float dt = 1.0f / 60.0f;
    int substeps = 8;

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

    void pull_particles(Vector2 position);

    void push_particles(Vector2 position);

private:
    void apply_gravity();
    void apply_circular_boundary();
    void update_particles(float dt);
    void resolve_collisions();
};
