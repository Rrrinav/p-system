#pragma once
#include <algorithm>
#include <vector>

#include "./particle.hpp"

#include "../raylib/include/raymath.h"

struct Link
{
  int particle1;      // First particle index
  int particle2;      // Second particle index
  float rest_length;  // Link's natural length
  float stiffness;    // How rigid the link is (0-1)
  bool active;        // Whether the link is currently active
  Link(int a, int b, float length, float stiff = 0.8f) : particle1(a), particle2(b), rest_length(length), stiffness(stiff), active(true) {}
};

class Link_System
{
  std::vector<Link> links;           // Links between particles
  std::vector<Particle> &particles;  // Reference to particles

public:
  Link_System(std::vector<Particle> &particles) : particles(particles) {}

  int add_link(int particleA, int particleB, float stiffness = 0.8f)
  {
    float distance = Vector2Distance(particles[particleA].position, particles[particleB].position);
    float max_rad = particles[particleA].radius > particles[particleB].radius ? particles[particleA].radius : particles[particleB].radius;
    distance = std::clamp(distance, 0.0001f, 2 * max_rad + 2);  // Prevent division by zero
    links.emplace_back(particleA, particleB, distance, stiffness);
    return links.size() - 1;  // Return index of the new link
  }

  void create_chain(const std::vector<int> &particleIndices, float stiffness = 0.8f)
  {
    for (size_t i = 0; i < particleIndices.size() - 1; ++i) add_link(particleIndices[i], particleIndices[i + 1], stiffness);
  }

  void break_link(int linkIndex)
  {
    if (linkIndex >= 0 && linkIndex < (int)links.size()) links[linkIndex].active = false;
  }

  void update(int iterations = 2)
  {
    for (int iter = 0; iter < iterations; ++iter)
    {
      for (auto &link : links)
      {
        if (!link.active) continue;

        Particle &p1 = particles[link.particle1];
        Particle &p2 = particles[link.particle2];

        // Calculate current distance
        Vector2 direction = Vector2Subtract(p2.position, p1.position);
        float currentDistance = Vector2Length(direction);

        // Skip if particles are in the same position
        if (currentDistance < 0.0001f) continue;

        // Calculate correction
        float difference = (currentDistance - link.rest_length) / currentDistance;
        Vector2 correction = Vector2Scale(direction, difference * 0.5f * link.stiffness);
        if (p1.fixed_position && p2.fixed_position)
        {
          // Both fixed_position - no movement
          continue;
        }
        else if (p1.fixed_position)
        {
          // Only p1 is fixed_position - p2 takes all correction
          p2.position = Vector2Subtract(p2.position, Vector2Scale(correction, 2.0f));
        }
        else if (p2.fixed_position)
        {
          // Only p2 is fixed_position - p1 takes all correction
          p1.position = Vector2Add(p1.position, Vector2Scale(correction, 2.0f));
        }
        else
        {
          // Neither is fixed - split the correction
          p1.position = Vector2Add(p1.position, correction);
          p2.position = Vector2Subtract(p2.position, correction);
        }
      }
    }
  }

  void update_breaks(float breakThreshold = 1.5f)
  {
    for (auto &link : links)
    {
      if (!link.active) continue;

      const Particle &p1 = particles[link.particle1];
      const Particle &p2 = particles[link.particle2];

      float currentDistance = Vector2Distance(p1.position, p2.position);
      // Break if stretched too much
      if (currentDistance > link.rest_length * breakThreshold) link.active = false;
    }
  }

  int get_active_link_count() const
  {
    int count = 0;
    for (const auto &link : links)
      if (link.active) count++;
    return count;
  }

  void fix_particle(int particleIndex, bool fixed = true)
  {
    if (particleIndex >= 0 && particleIndex < (int)particles.size())
    {
      particles[particleIndex].set_fixed_position(fixed);
      particles[particleIndex].f_pos = particles[particleIndex].position;
    }
  }

  // Fix the first particle in a chain
  void fix_start(const std::vector<int> &chain, bool fixed = true)
  {
    if (!chain.empty()) fix_particle(chain.front(), fixed);
  }

  // Fix the last particle in a chain
  void fix_end(const std::vector<int> &chain, bool fixed = true)
  {
    if (!chain.empty()) fix_particle(chain.back(), fixed);
  }

  // Fix both ends of a chain
  void fix_both_ends(const std::vector<int> &chain, bool fixed = true)
  {
    fix_start(chain, fixed);
    fix_end(chain, fixed);
  }

  // Move a fixed particle to a new position
  void move_fixed_particle(int particleIndex, Vector2 newPosition)
  {
    if (particleIndex >= 0 && particleIndex < (int)particles.size() && particles[particleIndex].fixed_position)
    {
      particles[particleIndex].position = newPosition;
      particles[particleIndex].position_last = newPosition;  // Update position_last to prevent velocity
    }
  }
};
