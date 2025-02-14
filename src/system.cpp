#include "./system.hpp"

#include <algorithm>
#include <vector>

#include "../raylib/include/raymath.h"

Particle *Particle_system::add_particle(const Particle &particle)
{
    particles.emplace_back(particle);
    return &particles.back();
}

void Particle_system::apply_gravity()
{
    for (auto &particle : particles)
        particle.accelerate(gravity);
}

void Particle_system::update_particles(float dt)
{
    for (auto &cell : _grid)
        cell.clear();  // Clear grid before updating

    for (int i = 0; i < (int)particles.size(); ++i)
    {
        particles[i].update(dt);  // Update particle position

        int cell_id = get_cell_index(particles[i].position);
        if (cell_id >= 0 && cell_id < (int)_grid.size())
            _grid[cell_id].emplace_back(i);  // Add particle index to correct grid cell
    }
}

void Particle_system::set_cell_size(int x)
{
    int n_cell_size = x;
    int n_grid_width = screen_width / n_cell_size;
    int n_grid_height = screen_height / n_cell_size;
    int actual_cell_size = screen_width / n_grid_width;
    cell_size = actual_cell_size;
    this->inv_cell_size = 1.0f / actual_cell_size;
    grid_width = n_grid_width;
    grid_height = n_grid_height;
    _grid.resize(grid_width * grid_height);
}

void Particle_system::update(Bound_type bound_type)
{
    float sub_dt = dt / this->substeps;
    switch (bound_type)
    {
        case Bound_type::CIRCLE:
        {
            for (int i{this->substeps}; i; --i)
            {
                apply_gravity();
                update_particles(sub_dt);
                resolve_collisions();
                apply_circular_boundary();
            }
        }
        break;
        case Bound_type::SCREEN:
        {
            for (int i{this->substeps}; i; --i)
            {
                apply_gravity();
                update_particles(sub_dt);
                resolve_collisions();
                apply_bounds();
            }
        }
        break;
        default:
        {
            for (int i{this->substeps}; i; --i)
            {
                apply_gravity();
                resolve_collisions();
                apply_bounds();
                update_particles(sub_dt);
            }
        }
        break;
    }
}

std::vector<Particle> const &Particle_system::get_particles() const { return particles; }

void Particle_system::apply_circular_boundary()
{
    for (auto &particle : particles)
    {
        // Vector from center to particle
        Vector2 radius = Vector2Subtract(boundary_center, particle.position);
        float distance = Vector2Length(radius);
        // Check if particle is beyond boundary
        if (distance > (boundary_radius - particle.radius))
        {
            // Calculate normal (pointing inward)
            Vector2 normal = Vector2Scale(radius, 1.0f / distance);
            // Get current velocity (based on positions)
            Vector2 velocity = particle.get_velocity();
            // Move particle back along normal by penetration amount
            particle.position = Vector2Subtract(particle.position, Vector2Scale(normal, boundary_radius - particle.radius - distance));
            // Calculate reflected velocity
            float dot = Vector2DotProduct(velocity, normal);
            Vector2 reflection = Vector2Subtract(velocity, Vector2Scale(normal, 2.0f * dot));
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
    std::vector<int> cell_neighbours;
    cell_neighbours.reserve(4);

    for (int cell_id = 0; cell_id < (int)_grid.size(); ++cell_id)
    {
        auto &cell = _grid[cell_id];
        if (cell.empty())
            continue;

        get_neighbour_cells(cell_id, cell_neighbours);

        for (size_t i = 0; i < cell.size(); ++i)
        {
            Particle &p1 = particles[cell[i]];
            for (size_t j = 0; j < cell.size(); ++j)
            {
                if (i == j)
                    continue;
                resolve_single_collision(p1, particles[cell[j]]);
            }

            for (int neighbour_id : cell_neighbours)
            {
                if (neighbour_id < 0 || neighbour_id >= (int)_grid.size())
                    continue;
                auto &neighbour = _grid[neighbour_id];
                for (int p2_idx : neighbour)
                    resolve_single_collision(p1, particles[p2_idx]);
            }
        }
    }
}

void Particle_system::apply_bounds()
{
    for (auto &particle : this->particles)
    {
        // Out of bounds on left or right
        if (particle.position.x - particle.radius < 0 || particle.position.x + particle.radius > screen_width)
        {
            particle.position.x = std::clamp(particle.position.x, 0.0f + particle.radius, (float)screen_width - particle.radius);
            particle.set_velocity({-particle.get_velocity().x, particle.get_velocity().y}, 1);
        }

        // Out of bounds on top or bottom
        if (particle.position.y - particle.radius < 0 || particle.position.y + particle.radius > screen_height)
        {
            particle.position.y = std::clamp(particle.position.y, 0.0f + particle.radius, (float)screen_height - particle.radius);
            particle.set_velocity({particle.get_velocity().x, -particle.get_velocity().y}, 1);
        }
    }
}

int Particle_system::get_cell_index(Vector2 position) const
{
    int x = (position.x * inv_cell_size);
    int y = (position.y * inv_cell_size);
    x = std::clamp(x, 0, grid_width - 1);
    y = std::clamp(y, 0, grid_height - 1);
    return x + y * grid_width;
}

constexpr int neighbor_offsets[][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}};

void Particle_system::get_neighbour_cells(int cell_index, std::vector<int> &neighbours) const
{
    neighbours.clear();
    int x = cell_index % grid_width;
    int y = cell_index / grid_width;

    for (int i = 0; i < 4; ++i)
    {
        int new_x = x + neighbor_offsets[i][0];
        int new_y = y + neighbor_offsets[i][1];

        if (new_x >= 0 && new_x < grid_width && new_y >= 0 && new_y < grid_height)
            neighbours.push_back(new_x + new_y * grid_width);
    }
}

void Particle_system::resolve_single_collision(Particle &p1, Particle &p2)
{
    Vector2 direction = Vector2Subtract(p1.position, p2.position);
    float distance = Vector2Length(direction);
    float min_distance = p1.radius + p2.radius;

    if (distance < min_distance && distance > 0.0001f)
    {
        Vector2 normal = Vector2Normalize(direction);
        float overlap = 0.5f * (min_distance - distance);

        p1.position = Vector2Add(p1.position, Vector2Scale(normal, overlap * 0.5));
        p2.position = Vector2Subtract(p2.position, Vector2Scale(normal, overlap * 0.5));
    }
}
