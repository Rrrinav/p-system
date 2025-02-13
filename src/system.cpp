#include "./system.hpp"

Particle& Particle_system::add_particle(const Particle &particle)
{
   return particles.emplace_back(particle);
}

void Particle_system::apply_gravity()
{
    for (auto &particle : particles) particle.accelerate(gravity);
}

void Particle_system::update_particles()
{
    for (auto &particle : particles) particle.update(this->dt);
}

void Particle_system::update()
{
    apply_gravity();
    update_particles();
}

std::vector<Particle> const &Particle_system::get_particles() const
{
    return particles;
}
