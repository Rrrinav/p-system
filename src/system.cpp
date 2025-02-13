#include <algorithm>

#include "./system.hpp"
#include "../raylib/include/raymath.h"


Particle* Particle_system::add_particle(const Particle &particle)
{
   particles.emplace_back(particle);
   return &particles.back();
}

void Particle_system::apply_gravity()
{
    for (auto &particle : particles) particle.accelerate(gravity);
}

void Particle_system::update_particles(float dT)
{
    for (auto &particle : particles) particle.update(dT);
}

void Particle_system::update()
{
    float sub_dt = dt / substeps;
    for (int i{substeps};i; --i)
    {
        apply_gravity();
        resolve_collisions();
        apply_circular_boundary();
        update_particles(sub_dt);
    }
}

std::vector<Particle> const &Particle_system::get_particles() const
{
    return particles;
}


void Particle_system::apply_circular_boundary()
{
    for (auto & particle : particles)
    {
        // Vector from center to particle
        Vector2 radius = Vector2Subtract(boundary_center, particle.position);
        float distance = Vector2Length(radius);
        // Check if particle is beyond boundary
        if (distance > (boundary_radius - particle.radius))
        {
            // Calculate normal (pointing inward)
            Vector2 normal = Vector2Scale(radius, 1.0f/distance);
            // Get current velocity (based on positions)
            Vector2 velocity = particle.get_velocity();
            // Move particle back along normal by penetration amount
            particle.position = Vector2Subtract( particle.position, Vector2Scale(normal, boundary_radius - particle.radius  - distance));
            // Calculate reflected velocity
            float dot = Vector2DotProduct(velocity, normal);
            Vector2 reflection = Vector2Subtract( velocity, Vector2Scale(normal, 2.0f * dot));
            // Apply reflection by modifying position_last
            // position_last needs to be set such that position - position_last = reflected_velocity
            particle.set_velocity(reflection, 1);
        }
    }
}


void Particle_system::pull_particles(Vector2 position)
{
    for (auto &particle : particles)
    {
        Vector2 direction = Vector2Subtract(position, particle.position);
        float distance = Vector2Length(direction);
        particle.accelerate(direction * std::max(0.0f, 10 * (150 - distance)));
    }
}

void Particle_system::push_particles(Vector2 position)
{
    for (auto &particle : particles)
    {
        Vector2 direction = Vector2Subtract(particle.position, position);
        float distance = Vector2Length(direction);
        particle.accelerate(direction * std::max(0.0f, 10 * (150 - distance)));
    }
}


void Particle_system::resolve_collisions()
{
    int n = particles.size();
    for (int i = 0; i < n; ++i)
    {
        Particle &particle_1 = particles[i];
        for (int j = i + 1; j < n; ++j)
        {
            if (i == j) continue;
            Particle &particle_2 = particles[j];
            Vector2 direction = Vector2Subtract(particle_1.position, particle_2.position);
            float distance = Vector2Length(direction);
            float min_distance = particle_1.radius + particle_2.radius;
            if (distance < min_distance)
            {
                Vector2 normal = Vector2Normalize(direction);
                float total_mass = particle_1.radius * particle_1.radius + particle_2.radius * particle_2.radius;
                float mass_ratio = particle_1.radius * particle_1.radius / total_mass;
                float overlap = 0.5f * (min_distance - distance);
                particle_1.position = Vector2Add(particle_1.position, Vector2Scale(normal, overlap * (1 - mass_ratio)));
                particle_2.position = Vector2Subtract(particle_2.position, Vector2Scale(normal, overlap * (mass_ratio)));
            }
        }
    }

}
