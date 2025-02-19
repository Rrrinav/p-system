#include <string>

#include "./raylib/include/raylib.h"
#include "./raylib/include/rlgl.h"
#include "./src/system.hpp"

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

int main()
{
    Color CIRCLE_COLOR = Color{20, 20, 20, 255};
    constexpr int S_WIDTH  = 1000;
    constexpr int S_HEIGHT = 900;
    constexpr int BOUNDARY_RADIUS = 400;
    constexpr int MAX_PARTICLES = 35000;
    constexpr int TEXTURE_SIZE = 32;  // Size of the particle texture
    static int frame_count = 0;

    InitWindow(S_WIDTH, S_HEIGHT, "PP: Particles and particles");

    // Generate white circle texture
    Texture2D circleTexture = GenerateCircleTexture(TEXTURE_SIZE / 2, WHITE);

    Particle_system system;
    system.boundary_center = {(float)S_WIDTH / 2, (float)S_HEIGHT / 2};
    system.boundary_radius = BOUNDARY_RADIUS;
    system.screen_width = S_WIDTH;
    system.screen_height = S_HEIGHT;
    system.set_cell_size(9);
    system.variable_radius = false;
    int particle_count = 0;
    constexpr int r = 3;
    system.reserve(MAX_PARTICLES);

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

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            system.pull_particles(GetMousePosition());
        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
            system.push_particles(GetMousePosition());
        if (particle_count < MAX_PARTICLES && (frame_count & 1) == 0)
        {
            int hue = (int)(frame_count % 360);
            system.add_particle(Particle(4 * r, 60, 500000, 0, ColorFromHSV(hue, 0.8, 0.6), r));
            system.add_particle(Particle(4 * r, 120, 500000, 0, ColorFromHSV(hue, 0.8, 0.6), r));
            system.add_particle(Particle(4 * r, 180, 500000, 0, ColorFromHSV(hue, 0.8, 0.6), r));
            system.add_particle(Particle(4 * r, 240, 500000, 0, ColorFromHSV(hue, 0.8, 0.6), r));
            system.add_particle(Particle(4 * r, 300, 500000, 0, ColorFromHSV(hue, 0.8, 0.6), r));
            system.add_particle(Particle(4 * r, 360, 500000, 0, ColorFromHSV(hue, 0.8, 0.6), r));
            particle_count += 6;
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

