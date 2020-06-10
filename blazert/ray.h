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
  const Vec3r<T> direction;
  const Vec3r<T> direction_inv;
  const Vec3ui direction_sign;
  T min_hit_distance;
  T max_hit_distance;
  bool cull_back_face;
  bool any_hit;

public:
  Ray() = delete;
  Ray(const Vec3r<T> &origin, const Vec3r<T> &direction, T min_hit_distance = T(0.),
      T max_hit_distance = std::numeric_limits<T>::max(), bool cull_back_face = false, bool any_hit = false)
      : origin(origin), direction(normalize(direction)), direction_inv((static_cast<T>(1.)/direction)),  // TODO: maybe normalize on creation?
        direction_sign{static_cast<unsigned int>(direction[0] < static_cast<T>(0.0) ? 1 : 0),
                        static_cast<unsigned int>(direction[1] < static_cast<T>(0.0) ? 1 : 0),
                        static_cast<unsigned int>(direction[2] < static_cast<T>(0.0) ? 1 : 0)},
        min_hit_distance(min_hit_distance), max_hit_distance(max_hit_distance),
        cull_back_face(cull_back_face), any_hit(any_hit) {}
};

template<typename T>
struct BLAZERTALIGN RayHit {
  Vec3r<T> normal;
  Vec2r<T> uv;
  T hit_distance = std::numeric_limits<T>::max();
  unsigned int prim_id = -1;
  unsigned int geom_id = -1;
};

}// namespace blazert

#endif// BLAZERT_RAY_H_
