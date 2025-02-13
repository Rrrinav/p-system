#include "./raylib/include/raylib.h"
#include "./src/particle.hpp"
#include "./src/system.hpp"

int main()
{
    Color CL_COLOR     = Color{120, 120, 120, 255};
    Color CIRCLE_COLOR = Color{20, 20, 20, 255};

    constexpr int S_WIDTH = 800;
    constexpr int S_HEIGHT = 600;

    InitWindow(S_WIDTH, S_HEIGHT, "PP: Particles and particles");
    SetTargetFPS(60);

    Particle_system system;
    system.boundary_center = {(float)S_WIDTH / 2, (float)S_HEIGHT / 2};
    system.add_particle(Particle(((float)S_WIDTH / 2),  20, 0, 0, RED, 10));

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(CL_COLOR);
        DrawCircle(S_WIDTH / 2, S_HEIGHT / 2, 250, CIRCLE_COLOR);
        const auto &particles = system.get_particles();
        for (const auto &particle : particles)
        {
            DrawCircleV(particle.position, particle.radius, particle.color);
        }
        system.update();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
