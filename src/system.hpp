#pragma once

#include <vector>

#include "./chain_links.hpp"
#include "./particle.hpp"

class Particle_system
{
  Vector2 gravity{0.0f, 400.0f};
  float dt = 1.0f / 60.0f;
  int substeps = 8;
  int grid_width = 0;
  int grid_height = 0;
  float inv_cell_size = 0.0f;
  std::vector<Particle> particles;
  std::vector<std::vector<int>> _grid;
  Link_System links;

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

  Particle_system() : particles(), _grid(), links(particles) {}
  Particle_system(Particle_system const &other) = delete;
  Particle_system &operator=(Particle_system const &other) = delete;
  Particle_system(Particle_system &&other) = delete;
  Particle_system &operator=(Particle_system &&other) = delete;

  Particle *add_particle(const Particle &particle);

  void update(Bound_type bound_type = Bound_type::SCREEN);

  std::vector<Particle> const &get_particles() const;
  void reserve(int n) { this->particles.reserve(n); }

  void pull_particles(Vector2 position);

  void push_particles(Vector2 position);

  Vector2 get_gravity() const { return gravity; }
  void set_gravity(Vector2 new_gravity) { gravity = new_gravity; }
  void set_cell_size(int x);

  std::vector<int> create_particle_chain(Vector2 start, Vector2 end, int count, float radius, Color color)
  {
    std::vector<int> indices;
    indices.reserve(count);

    for (int i = 0; i < count; ++i)
    {
      float t = (float)i / (count - 1);
      Vector2 pos = {start.x + t * (end.x - start.x), start.y + t * (end.y - start.y)};

      // Add particle and remember its index
      Particle p(pos, {0, 0}, color, radius);
      add_particle(p);
      indices.push_back(particles.size() - 1);
    }

    // Create links between the particles
    links.create_chain(indices);

    return indices;
  }

  void update_links(int iterations)
  {
    links.update(iterations);
    links.update_breaks();
  }

  void fix_particle(int index, bool fixed) { links.fix_particle(index, fixed); }

  void fix_chain_start(const std::vector<int> &chain, bool fixed) { links.fix_start(chain, fixed); }

  void fix_chain_end(const std::vector<int> &chain, bool fixed) { links.fix_end(chain, fixed); }

  void fix_chain_both_ends(const std::vector<int> &chain, bool fixed) { links.fix_both_ends(chain, fixed); }

  void move_fixed_particle(int index, Vector2 newPosition) { links.move_fixed_particle(index, newPosition); }

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
