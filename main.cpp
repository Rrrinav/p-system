#include <sys/types.h>

#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "./raylib/include/raylib.h"
#include "./raylib/include/rlgl.h"
#include "./src/system.hpp"

#define ASSERT(condition, message)                                                                                       \
  do {                                                                                                                   \
    if (!(condition))                                                                                                    \
    {                                                                                                                    \
      std::cerr << __FILE__ << ":" << __LINE__ << ": error" << "Assertion failed: " << (message) << " in " << std::endl; \
      std::exit(EXIT_FAILURE);                                                                                           \
    }                                                                                                                    \
  } while (false)

// Generate a circle texture
Texture2D GenerateCircleTexture(int radius, Color color = WHITE)
{
  Image img = GenImageColor(radius * 2, radius * 2, BLANK);

  // Draw a white circle on transparent background
  ImageDrawCircle(&img, radius, radius, radius, color);

  // Generate texture from image
  Texture2D texture = LoadTextureFromImage(img);
  UnloadImage(img);

  return texture;
}

std::array<std::string, 3> split(const std::string &s, char delim)
{
  std::array<std::string, 3> tokens;
  std::stringstream ss(s);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, delim)) tokens[i++] = item;
  return tokens;
}

int main()
{
  Color CIRCLE_COLOR = Color{20, 20, 20, 255};
  constexpr int S_WIDTH = 1000;
  constexpr int S_HEIGHT = 900;
  constexpr int BOUNDARY_RADIUS = 400;
  constexpr int MAX_PARTICLES = 32000;
  constexpr int TEXTURE_SIZE = 32;  // Size of the particle texture
  static int frame_count = 0;

  InitWindow(S_WIDTH, S_HEIGHT, "PP: Particles and particles");
  //SetTargetFPS(60);

  // Generate white circle texture
  Texture2D circleTexture = GenerateCircleTexture(TEXTURE_SIZE / 2, WHITE);

  Particle_system system;
  system.boundary_center = {(float)S_WIDTH / 2, (float)S_HEIGHT / 2};
  system.boundary_radius = BOUNDARY_RADIUS;
  system.screen_width = S_WIDTH;
  system.screen_height = S_HEIGHT;
  system.set_cell_size(14);
  system.variable_radius = false;
  int particle_count = 0;
  constexpr int r = 6;
  system.reserve(MAX_PARTICLES);
  std::vector<Color> colors;
  //std::vector<int> chain1 = system.create_particle_chain({100, 100}, {300, 200}, 10, 8.0f, RED);
  //
  //std::vector<int> chain2 = system.create_particle_chain({400, 100}, {600, 200}, 15, 6.0f, BLUE);

  // 2. Bridge chain (fixed at both ends)
  std::vector<int> bridgeChain = system.create_particle_chain({400, 500}, {700, 500}, 20, 8.0f, BLUE);
  system.fix_chain_both_ends(bridgeChain, true);  // Fix both ends

  std::ifstream file("./pp_color.txt");

  for (std::string line; std::getline(file, line);)
  {
    u_char r, g, b;
    auto tokens = split(line, ' ');
    ASSERT(tokens.size() == 3, "Color has to have 3 values");
    for (int i = 0; i < 3; i++)
    {
      if (i == 0) r = std::stoi(tokens[i]);
      if (i == 1) g = std::stoi(tokens[i]);
      if (i == 2) b = std::stoi(tokens[i]);
    }
    colors.push_back(Color{r, g, b, 255});
  }

  while (!WindowShouldClose())
  {
    BeginDrawing();
    ClearBackground(CIRCLE_COLOR);

    const auto &particles = system.get_particles();

    // Draw each particle as a textured quad
    for (const auto &particle : particles)
    {
      // Calculate the position and size for the texture
      float scale = particle.radius * 2.0f / TEXTURE_SIZE;

      // Use the particle's color to tint the white texture
      DrawTextureEx(circleTexture, {particle.position.x - particle.radius, particle.position.y - particle.radius},
                    0.0f,  // rotation
                    scale, particle.color);
    }

    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) system.pull_particles(GetMousePosition());
    if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) system.push_particles(GetMousePosition());

    if (particle_count < MAX_PARTICLES && (frame_count % 5) == 0)
    {
      Color color = colors[particle_count++];
      system.add_particle(Particle((float)S_WIDTH / 2, 100, 0, 100, color, r));
      //system.add_particle(Particle(4 * r, 60, 500000, 0, color, r));
      //color = colors[particle_count++];
      //system.add_particle(Particle(4 * r, 120, 500000, 0, color, r));
      //color = colors[particle_count++];
      //system.add_particle(Particle(4 * r, 180, 500000, 0, color, r));
      //color = colors[particle_count++];
      //system.add_particle(Particle(4 * r, 240, 500000, 0, color, r));
      //color = colors[particle_count++];
      //system.add_particle(Particle(4 * r, 300, 500000, 0, color, r));
      //color = colors[particle_count++];
      //system.add_particle(Particle(4 * r, 360, 500000, 0, color, r));
    }
    frame_count++;

    system.update();

    std::string fps = std::to_string(GetFPS());
    DrawText(fps.c_str(), 10, 10, 20, RED);
    DrawText(std::to_string(particle_count).c_str(), 10, 30, 20, RED);

    EndDrawing();
  }

  // Clean up
  UnloadTexture(circleTexture);
  CloseWindow();
  return 0;
}
