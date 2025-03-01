#pragma once

#include "../raylib/include/raylib.h"

struct Particle
{
  Vector2 position;
  Vector2 f_pos;
  Vector2 position_last;
  Vector2 acceleration;
  Color   color;
  float   radius{10};
  int     cellID{-1};
  bool    fixed_position{false};
  bool    fixed_velocity{false};

  Particle() = default;
  Particle(Vector2 position, Vector2 acceleration, Color color, float radius)
      : position(position), position_last(position), acceleration(acceleration), color(color), radius(radius)
  {
  }
  Particle(float x, float y, float acceleration_x, float acceleration_y, Color color, float radius)
      : position({x, y}), position_last({x, y}), acceleration({acceleration_x, acceleration_y}), color(color), radius(radius)
  {
  }
  Particle(Particle const &other)
      : position(other.position),
        position_last(other.position_last),
        acceleration(other.acceleration),
        color(other.color),
        radius(other.radius)
  {
  }
  Particle &operator=(Particle const &other)
  {
    position = other.position;
    position_last = other.position_last;
    acceleration = other.acceleration;
    color = other.color;
    radius = other.radius;
    return *this;
  }
  Particle(Particle &&other)
  {
    this->position = other.position;
    this->position_last = other.position_last;
    this->acceleration = other.acceleration;
    this->color = other.color;
    this->radius = other.radius;
  }
  Particle &operator=(Particle &&other)
  {
    this->position = other.position;
    this->position_last = other.position_last;
    this->acceleration = other.acceleration;
    this->color = other.color;
    this->radius = other.radius;
    return *this;
  }

  void update(float dt);

  void accelerate(const Vector2 &force);

  void set_velocity(const Vector2 &v, float dt);

  void add_velocity(const Vector2 &v, float dt);

  void set_fixed_position(bool fix) { this->fixed_position = fix; }
  void set_fixed_velocity(bool fix) { this->fixed_velocity = fix; }

  Vector2 get_velocity() const;
};
