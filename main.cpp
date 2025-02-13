#include <cmath>

#include "./raylib/include/raylib.h"
#include "./src/particle.hpp"
#include "./src/system.hpp"

int main()
{
    Color CL_COLOR     = Color{120, 120, 120, 255};
    Color CIRCLE_COLOR = Color{20, 20, 20, 255};

    constexpr int S_WIDTH  = 900;
    constexpr int S_HEIGHT = 900;
    constexpr int BOUNDARY_RADIUS = 400;
    constexpr int MAX_PARTICLES = 100;
    InitWindow(S_WIDTH, S_HEIGHT, "PP: Particles and particles");
    SetTargetFPS(60);

    Particle_system system;
    system.boundary_center = {(float)S_WIDTH / 2, (float)S_HEIGHT / 2};
    system.boundary_radius = BOUNDARY_RADIUS;

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(CL_COLOR);
        DrawCircle(S_WIDTH / 2, S_HEIGHT / 2, BOUNDARY_RADIUS, CIRCLE_COLOR);
        const auto &particles = system.get_particles();
        for (const auto &particle : particles)
        {
            DrawCircleV(particle.position, particle.radius, particle.color);
        }

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
        {
            system.pull_particles(GetMousePosition());
        }

        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
        {
            system.push_particles(GetMousePosition());
        }

        if (particles.size() < MAX_PARTICLES && fmod(GetTime(), 0.1) < GetFrameTime())
        {

            system.add_particle(Particle(((float)S_WIDTH / 2),  200, 100000, 0, BEIGE, 10));
        }

        system.update();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
