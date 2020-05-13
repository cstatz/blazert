#pragma once
#ifndef BLAZERT_RAY_H_
#define BLAZERT_RAY_H_

#include <limits>

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

template<typename T>
class BLAZERTALIGN Ray {
public:
  const Vec3r<T> origin;
  const Vec3r<T> direction;  // Direction needs to be always normalized. We ensure this in the constructor.
  T min_hit_distance;
  T max_hit_distance;

public:
  Ray() = delete;
  Ray(const Vec3r<T> &origin, const Vec3r<T> &direction, T min_hit_distance = T(0.), T max_hit_distance = std::numeric_limits<T>::max())
      : origin(origin), direction(normalize(direction)), min_hit_distance(min_hit_distance), max_hit_distance(max_hit_distance) {};
};

template<typename T>
struct BLAZERTALIGN RayHit {
  Vec3r<T> normal;
  Vec2r<T> uv;
  T hit_distance = std::numeric_limits<T>::max();
  unsigned int prim_id = -1;
};

}// namespace blazert

#endif// BLAZERT_RAY_H_
