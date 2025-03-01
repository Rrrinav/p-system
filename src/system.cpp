#include "./system.hpp"

#include <algorithm>
#include <thread>
#include <vector>

#include "../raylib/include/raymath.h"
#include "./thread.hpp"

Particle *Particle_system::add_particle(const Particle &particle)
{
  particles.emplace_back(particle);
  Particle *p = &particles.back();
  int cell_id = get_cell_index(p->position);
  p->cellID = cell_id;
  return p;
}

void Particle_system::apply_gravity()
{
  for (Particle &p : particles)
  {
    if (!p.fixed_position) p.accelerate(gravity);
  }
}

inline void Particle_system::update_particles(float dt)
{
  for (auto &cell : _grid) cell.clear();

  for (int i = 0; i < (int)particles.size(); ++i)
  {
    particles[i].update(dt);
    int new_cell_id = get_cell_index(particles[i].position);
    if (new_cell_id >= 0 && new_cell_id < (int)_grid.size()) _grid[new_cell_id].push_back(i);
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
    [[unlikely]] case Bound_type::CIRCLE:
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
    [[likely]] case Bound_type::SCREEN:
    {
      for (int i{this->substeps}; i; --i)
      {
        apply_gravity();
        update_particles(sub_dt);
        update_links(2);
        resolve_collisions();
        apply_bounds();
      }
    }
    break;
    [[likely]] default:
    {
      for (int i{this->substeps}; i; --i)
      {
        apply_gravity();
        update_particles(sub_dt);
        update_links(2);
        apply_bounds();
        resolve_collisions();
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

inline void Particle_system::resolve_collisions()
{
  int num_threads = std::thread::hardware_concurrency();
  int cols_per_thread = grid_width / num_threads;
  int extra_cols = grid_width % num_threads;  // Handle uneven division

  Multi_threader thread_pool(num_threads);

  for (int pass = 0; pass < 2; ++pass)
  {
    for (int t = 0; t < num_threads; ++t)
    {
      // Compute thread's total column range
      int full_start = t * cols_per_thread + std::min(t, extra_cols);
      int full_end = full_start + cols_per_thread + (t < extra_cols ? 1 : 0);

      // Ensure non-empty range
      if (full_start >= grid_width) continue;

      // Split range for each pass
      int mid = (full_start + full_end) / 2;
      int start_col = (pass == 0) ? full_start : mid;
      int end_col = (pass == 0) ? mid : full_end;

      thread_pool.add_task(
          [this, start_col, end_col]()
          {
            std::vector<int> cell_neighbours;
            cell_neighbours.reserve(4);

            for (int col = start_col; col < end_col; ++col)
            {
              for (int row = 0; row < grid_height; ++row)
              {
                int cell_id = col + row * grid_width;
                if (cell_id < 0 || cell_id >= (int)_grid.size()) continue;

                auto &cell = _grid[cell_id];

                get_neighbour_cells(cell_id, cell_neighbours);

                for (size_t i = 0; i < cell.size(); ++i)
                {
                  Particle &p1 = particles[cell[i]];

                  // Collisions within the same cell
                  for (size_t j = i + 1; j < cell.size(); ++j) resolve_single_collision(p1, particles[cell[j]]);

                  // Collisions with neighboring cells
                  for (int neighbour_id : cell_neighbours)
                  {
                    if (neighbour_id < 0 || neighbour_id >= (int)_grid.size()) continue;
                    auto &neighbour = _grid[neighbour_id];
                    for (int index : neighbour) resolve_single_collision(p1, particles[index]);
                  }
                }
              }
            }
          });
    }
    thread_pool.wait();  // Ensure all threads complete before the next pass
  }
}

void Particle_system::apply_bounds()
{
  for (auto &particle : this->particles)
  {
    if (particle.position.x - particle.radius < 0 || particle.position.x + particle.radius > screen_width)
    {
      particle.position.x = std::clamp(particle.position.x, particle.radius, screen_width - particle.radius);
      particle.set_velocity({-particle.get_velocity().x, particle.get_velocity().y}, 1);
    }

    if (particle.position.y - particle.radius < 0 || particle.position.y + particle.radius > screen_height)
    {
      particle.position.y = std::clamp(particle.position.y, particle.radius, screen_height - particle.radius);
      particle.set_velocity({particle.get_velocity().x, -particle.get_velocity().y}, 1);
    }
  }
  return;

  int threshold = 2;

  // Top edge (also handles top-left corner)
  auto top = [&]()
  {
    for (int x = 0; x < grid_width; x++)
    {
      for (int y = 0; y < threshold; y++)
      {
        for (auto &particle : _grid[x + y * grid_width])
        {
          Particle &p = particles[particle];
          if (p.position.y - p.radius < 0)
          {
            p.position.y = p.radius;
            p.set_velocity({p.get_velocity().x, -p.get_velocity().y}, 1);
          }
        }
      }
    }
  };

  // Bottom edge (also handles bottom-right corner)
  auto bottom = [&]()
  {
    for (int x = 0; x < grid_width; x++)
    {
      for (int y = std::max(0, grid_height - threshold); y < grid_height; y++)
      {
        for (auto &particle : _grid[x + y * grid_width])
        {
          Particle &p = particles[particle];
          if (p.position.y + p.radius > screen_height)
          {
            p.position.y = screen_height - p.radius;
            p.set_velocity({p.get_velocity().x, -p.get_velocity().y}, 1);
          }
        }
      }
    }
  };

  // Left edge (also handles bottom-left corner)
  auto left = [&]()
  {
    for (int y = 0; y < grid_height; y++)
    {
      for (int x = 0; x < threshold; x++)
      {
        for (auto &particle : _grid[x + y * grid_width])
        {
          Particle &p = particles[particle];
          if (p.position.x - p.radius < 0)
          {
            p.position.x = p.radius;
            p.set_velocity({-p.get_velocity().x, p.get_velocity().y}, 1);
          }
        }
      }
    }
  };

  // Right edge (also handles top-right corner)
  auto right = [&]()
  {
    for (int y = 0; y < grid_height; y++)
    {
      for (int x = std::max(0, grid_width - threshold); x < grid_width; x++)
      {
        for (auto &particle : _grid[x + y * grid_width])
        {
          Particle &p = particles[particle];
          if (p.position.x + p.radius > screen_width)
          {
            p.position.x = screen_width - p.radius;
            p.set_velocity({-p.get_velocity().x, p.get_velocity().y}, 1);
          }
        }
      }
    }
  };

  // Spawn 4 threads for independent edge handling
  std::thread t1(top);
  std::thread t2(bottom);
  std::thread t3(left);
  std::thread t4(right);

  t1.join();
  t2.join();
  t3.join();
  t4.join();
}

inline int Particle_system::get_cell_index(Vector2 position) const
{
  int x = (position.x * inv_cell_size);
  int y = (position.y * inv_cell_size);
  return x + y * grid_width;
}

constexpr int neighbor_offsets[][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}};

inline void Particle_system::get_neighbour_cells(int cell_index, std::vector<int> &neighbours) const
{
  neighbours.clear();
  int x = cell_index % grid_width;
  int y = cell_index / grid_width;

  for (int i = 0; i < 4; ++i)
  {
    int new_x = x + neighbor_offsets[i][0];
    int new_y = y + neighbor_offsets[i][1];

    if (new_x >= 0 && new_x < grid_width && new_y >= 0 && new_y < grid_height) neighbours.push_back(new_x + new_y * grid_width);
  }
}

inline void Particle_system::resolve_single_collision(Particle &p1, Particle &p2)
{
  Vector2 direction = Vector2Subtract(p1.position, p2.position);
  float distance = Vector2Length(direction);
  float min_distance = p1.radius + p2.radius;

  if (distance < min_distance && distance > 0.0001f)
  {
    Vector2 normal = Vector2Normalize(direction);
    float overlap = 0.5f * (min_distance - distance);

    if (p1.fixed_position && p2.fixed_position) return;
    if (p1.fixed_position)
    {
      p2.position = Vector2Add(p2.position, Vector2Scale(normal, overlap));
      return;
    }
    if (p2.fixed_position)
    {
      p1.position = Vector2Subtract(p1.position, Vector2Scale(normal, overlap));
      return;
    }
    p1.position = Vector2Add(p1.position, Vector2Scale(normal, overlap * 0.5));
    p2.position = Vector2Subtract(p2.position, Vector2Scale(normal, overlap * 0.5));
  }
}
