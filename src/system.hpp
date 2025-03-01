#pragma once

#include <vector>

#include "./particle.hpp"

class Particle_system
{
  std::vector<Particle> particles;
  Vector2 gravity{0.0f, 400.0f};
  float dt = 1.0f / 60.0f;
  int substeps = 8;
  std::vector<std::vector<int>> _grid;
  int grid_width;
  int grid_height;
  float inv_cell_size;

public:
  enum class Bound_type
  {
    CIRCLE,
    SCREEN
  };
  int screen_width{};
  int screen_height{};
  int boundary_radius{250};
  Bound_type bound_type{Bound_type::SCREEN};
  Vector2 boundary_center{};
  int cell_size{25};
  bool variable_radius{false};

  Particle_system() = default;
  Particle_system(Particle_system const &other) = default;
  Particle_system &operator=(Particle_system const &other) = default;
  Particle_system(Particle_system &&other) = default;
  Particle_system &operator=(Particle_system &&other) = default;

  Particle *add_particle(const Particle &particle);

  void update(Bound_type bound_type = Bound_type::SCREEN);

  std::vector<Particle> const &get_particles() const;
  void reserve(int n) { this->particles.reserve(n); }

  void pull_particles(Vector2 position);

  void push_particles(Vector2 position);

  Vector2 get_gravity() const { return gravity; }
  void set_gravity(Vector2 new_gravity) { gravity = new_gravity; }
  void set_cell_size(int x);

private:
  inline void apply_gravity();
  inline void apply_circular_boundary();
  inline void apply_bounds();
  inline void update_particles(float dt);
  inline void resolve_collisions();
  inline int get_cell_index(Vector2 position) const;
  inline void get_neighbour_cells(int cell_index, std::vector<int> &neighbours) const;
  inline void resolve_single_collision(Particle &particle_1, Particle &particle_2);
};
