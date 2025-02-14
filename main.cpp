#include <cmath>

#include "./raylib/include/raylib.h"
#include "./src/particle.hpp"
#include "./src/system.hpp"

int main()
{
    Color CL_COLOR = Color{120, 120, 120, 255};
    Color CIRCLE_COLOR = Color{20, 20, 20, 255};
    constexpr int S_WIDTH = 900;
    constexpr int S_HEIGHT = 900;
    constexpr int BOUNDARY_RADIUS = 400;
    constexpr int MAX_PARTICLES = 500;

    InitWindow(S_WIDTH, S_HEIGHT, "PP: Particles and particles");
    SetTargetFPS(60);

    Particle_system system;
    system.boundary_center = {(float)S_WIDTH / 2, (float)S_HEIGHT / 2};
    system.boundary_radius = BOUNDARY_RADIUS;
    system.screen_width = S_WIDTH;
    system.screen_height = S_HEIGHT;

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(CIRCLE_COLOR);
        //DrawCircle(S_WIDTH / 2, S_HEIGHT / 2, BOUNDARY_RADIUS, CIRCLE_COLOR);

        const auto &particles = system.get_particles();

        for (const auto &particle : particles)
            DrawCircleV(particle.position, particle.radius, particle.color);

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            system.pull_particles(GetMousePosition());
        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
            system.push_particles(GetMousePosition());
        if (IsKeyPressed(KEY_R))
        {
            Vector2 gravity = system.get_gravity();
            gravity.y *= -1;
            system.set_gravity(gravity);
        }

        if (particles.size() < MAX_PARTICLES && fmod(GetTime(), 0.08) < GetFrameTime())
        {
            Vector2 position = {float(S_WIDTH) / 2, float(S_HEIGHT) / 2 - 250};
            int r = GetRandomValue(8, 12);
            // Incremental color using HSV
            float c_wave = sin(GetTime() * 0.2f);
            if (c_wave < 0) c_wave *= -1;

            int hue = (int)(c_wave * 360);
            Particle *p = system.add_particle(Particle(position, {0, 0}, ColorFromHSV(hue, 0.8, 0.6), r));

            float wave = M_PI * 0.5f + sin(GetTime() * 3.0f);
            Vector2 direction = {float(cos(wave)), float(sin(wave))};
            p->set_velocity(direction, 1.0f);
        }

        system.update();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
