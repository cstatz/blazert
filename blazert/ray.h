#pragma once
#ifndef BLAZERT_RAY_H_
#define BLAZERT_RAY_H_

#include <limits>

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

/**
 * @brief
 *  Ray<T> defines a ray for the ray tracing.
 *
 * @details
 *
 * @tparam T floating point type
 */
template<typename T>
class BLAZERTALIGN Ray {
public:
  enum class Culling { None, Backface };
  enum class RegisterHit { Closest, Any };

public:
  const Vec3r<T> origin;
  const Vec3r<T> direction;
  const Vec3r<T> direction_inv;
  const Vec3ui direction_sign;
  T min_hit_distance;
  T max_hit_distance;
  Culling cull_back_face;
  RegisterHit any_hit;

public:
  Ray() = delete;
  /**
   * @brief Constructor for a Ray<T> to be used in ray tracing.
   *
   * @details
   *    A lot of additional information is calculated in the constructor which is of interest in the build or traversal
   *    of the BVH.
   *
   * @param origin Starting point from where the ray is launched.
   * @param direction Dircetion in which the ray is launched.
   * @param min_hit_distance  Minimum length the ray needs to have (default = 0).
   * @param max_hit_distance Maximum length the ray can have (default = std::numeric_limits<T>::max()).
   * @param cull_back_face If set to blazert::Ray<T>::Culling::Backface, culling back faces will be used (default = None).
   * @param any_hit If set to blazert::Ray<T>::RegisteredHit::Any, the first hit found in the traversal will register as the hit, which might not be the hit (default = Closest) closest to the ray origin.
   *
   * @todo backface culling is no implemented yet.
   *
   */
  Ray(const Vec3r<T> &origin, const Vec3r<T> &direction, T min_hit_distance = T(0.),
      T max_hit_distance = std::numeric_limits<T>::max(), Culling cull_back_face = Culling::None,
      RegisterHit any_hit = RegisterHit::Closest)
      : origin{origin}, direction{normalize(direction)},
        direction_inv{(static_cast<T>(1.) / direction)},// TODO: maybe normalize on creation?
        direction_sign{static_cast<unsigned int>(direction[0] < static_cast<T>(0.0) ? 1 : 0),
                       static_cast<unsigned int>(direction[1] < static_cast<T>(0.0) ? 1 : 0),
                       static_cast<unsigned int>(direction[2] < static_cast<T>(0.0) ? 1 : 0)},
        min_hit_distance(min_hit_distance), max_hit_distance(max_hit_distance), cull_back_face(cull_back_face),
        any_hit(any_hit) {}
};

/**
 * @brief RayHit structure holds information about an intersection between a ray and an object
 * @tparam T floating point type
 */
template<typename T>
struct BLAZERTALIGN RayHit {

  /// normal vector on surface at the intersection point
  Vec3r<T> normal;
  /// uv coordinates
  Vec2r<T> uv;
  /// distance between ray origin and the intersection point
  T hit_distance = std::numeric_limits<T>::max();
  /// primitive id of the hit object (= which exact primitive was it)
  unsigned int prim_id = static_cast<unsigned int>(-1);
  /// geometry id of the hit object (= which kind of geometry was it, e.g. sphere, triangle)
  unsigned int geom_id = static_cast<unsigned int>(-1);
};

}// namespace blazert

#endif// BLAZERT_RAY_H_
