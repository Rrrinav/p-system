// particle_link.hpp
#pragma once
#include <vector>

#include "./particle.hpp"
#include "../raylib/include/raymath.h"

struct Link
{
  int particleA;     // Index of first particle
  int particleB;     // Index of second particle
  float restLength;  // Natural length of the link
  float stiffness;   // How rigid the link is (0-1)
  bool active;       // Whether the link is currently active

  Link(int a, int b, float length, float stiff = 0.8f) : particleA(a), particleB(b), restLength(length), stiffness(stiff), active(true) {}
};

class Link_System
{
  std::vector<Link> links;
  std::vector<Particle> &particles;  // Reference to particles

public:
  Link_System(std::vector<Particle> &particles) : particles(particles) {}

  // Add a new link between two particles
  int add_link(int particleA, int particleB, float stiffness = 0.8f)
  {
    float distance = Vector2Distance(particles[particleA].position, particles[particleB].position);
    links.emplace_back(particleA, particleB, distance, stiffness);
    return links.size() - 1;  // Return index of the new link
  }

  // Create a chain of links between consecutive particles
  void create_chain(const std::vector<int> &particleIndices, float stiffness = 0.8f)
  {
    for (size_t i = 0; i < particleIndices.size() - 1; ++i) add_link(particleIndices[i], particleIndices[i + 1], stiffness);
  }

  // Break a specific link
  void break_link(int linkIndex)
  {
    if (linkIndex >= 0 && linkIndex < (int)links.size()) links[linkIndex].active = false;
  }

  // Update all links (apply constraints)
  void update(int iterations = 2)
  {
    for (int iter = 0; iter < iterations; ++iter)
    {
      for (auto &link : links)
      {
        if (!link.active) continue;

        Particle &p1 = particles[link.particleA];
        Particle &p2 = particles[link.particleB];

        // Calculate current distance
        Vector2 direction = Vector2Subtract(p2.position, p1.position);
        float currentDistance = Vector2Length(direction);

        // Skip if particles are in the same position
        if (currentDistance < 0.0001f) continue;

        // Calculate correction
        float difference = (currentDistance - link.restLength) / currentDistance;
        Vector2 correction = Vector2Scale(direction, difference * 0.5f * link.stiffness);

        // Apply correction to both particles
        p1.position = Vector2Add(p1.position, correction);
        p2.position = Vector2Subtract(p2.position, correction);
      }
    }
  }

  // Draw all links
  void draw() const
  {
    for (const auto &link : links)
    {
      if (!link.active) continue;

      const Particle &p1 = particles[link.particleA];
      const Particle &p2 = particles[link.particleB];

      // Draw as a line with thickness based on particles' radii
      float thickness = std::min(p1.radius, p2.radius) * 0.5f;
      // Use color slightly darker than particles
      Color lineColor = ColorBrightness(p1.color, -0.3f);
      DrawLineEx(p1.position, p2.position, thickness, lineColor);
    }
  }

  // Break links if they stretch too much
  void update_breaks(float breakThreshold = 1.5f)
  {
    for (auto &link : links)
    {
      if (!link.active) continue;

      const Particle &p1 = particles[link.particleA];
      const Particle &p2 = particles[link.particleB];

      float currentDistance = Vector2Distance(p1.position, p2.position);
      // Break if stretched too much
      if (currentDistance > link.restLength * breakThreshold) link.active = false;
    }
  }

  // Get the number of active links
  int get_active_link_count() const
  {
    int count = 0;
    for (const auto &link : links)
      if (link.active) count++;
    return count;
  }
};
