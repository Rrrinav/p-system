#pragma once

#include <unordered_map>
#include <vector>

#include "./particle.hpp"

class Particle_system
{
    std::vector<Particle> particles;
    Vector2 gravity{0.0f, 900.0f};
    float dt = 1.0f / 60.0f;
    int substeps = 8;
    std::unordered_map<int, std::vector<int>> _grid; // {cell: particle indices}
    int grid_width;
    int grid_height;

public:
    enum class Bound_type {
        CIRCLE, SCREEN
    };
    int screen_width{};
    int screen_height{};
    int boundary_radius{250};
    Bound_type bound_type{Bound_type::SCREEN};
    Vector2 boundary_center{};
    int cell_size{25};

    Particle_system() = default;
    Particle_system(Particle_system const &other) = default;
    Particle_system &operator=(Particle_system const &other) = default;
    Particle_system(Particle_system &&other) = default;
    Particle_system &operator=(Particle_system &&other) = default;

    Particle* add_particle(const Particle &particle);

    void update(Bound_type bound_type = Bound_type::SCREEN);

    std::vector<Particle> const &get_particles() const;

    void pull_particles(Vector2 position);

    void push_particles(Vector2 position);

    Vector2 get_gravity() const {return gravity;}
    void set_gravity(Vector2 new_gravity) {gravity = new_gravity;}
    void set_grid_size(int x);

private:
    void apply_gravity();
    void apply_circular_boundary();
    void apply_bounds();
    void update_particles(float dt);
    void resolve_collisions();
    int get_cell_index(Vector2 position) const;
    std::vector<int> get_neighbour_cells(int cell_index) const;
    void resolve_single_collision(Particle &particle_1, Particle &particle_2);
};
