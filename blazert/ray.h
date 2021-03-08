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
  enum class CullBackFace { yes, no };
  enum class AnyHit { yes, no };

public:
  const Vec3r<T> origin;
  const Vec3r<T> direction;
  const Vec3r<T> direction_inv;
  const Vec3ui direction_sign;
  T min_hit_distance;
  T max_hit_distance;
  CullBackFace cull_back_face;
  AnyHit any_hit;

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
   * @param cull_back_face If set to blazert::Ray<T>::CullBackFace::yes, culling backfaces will be used (default = no).
   * @param any_hit If set to blazert::Ray<T>::AnyHit::yes, the first hit found in the traversal will register as the hit, which might not be the hit (default = no) closest to the ray origin.
   *
   * @todo backface culling is no implemented yet.
   *
   *
   */
  Ray(const Vec3r<T> &origin_, const Vec3r<T> &direction_, T min_hit_distance_ = T(0.),
      T max_hit_distance_ = std::numeric_limits<T>::max(), CullBackFace cull_back_face_ = CullBackFace::no,
      AnyHit any_hit_ = AnyHit::no)
      : origin{origin_}, direction{normalize(direction_)},
        direction_inv{(static_cast<T>(1.) / direction)},// TODO: maybe normalize on creation?
        direction_sign{static_cast<unsigned int>(direction[0] < static_cast<T>(0.0) ? 1 : 0),
                       static_cast<unsigned int>(direction[1] < static_cast<T>(0.0) ? 1 : 0),
                       static_cast<unsigned int>(direction[2] < static_cast<T>(0.0) ? 1 : 0)},
        min_hit_distance(min_hit_distance_), max_hit_distance(max_hit_distance_), cull_back_face(cull_back_face_),
        any_hit(any_hit_) {}
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

template<typename T>
std::ostream &operator<<(std::ostream &stream, const Ray<T> &ray) {
  /// Conveniently output a single cylinder as JSON.
  stream << "{\n";

  stream << "  Ray: " << &ray << ",\n";
  stream << "  origin: [" << ray.origin[0] << "," << ray.origin[1] << "," << ray.origin[2] << "],\n";
  stream << "  direction: [" << ray.direction[0] << "," << ray.direction[1] << "," << ray.direction[2] << "],\n";
  stream << "  direction_inv: ["<< ray.direction_inv[0] << "," << ray.direction_inv[1] << "," << ray.direction_inv[2] << "],\n";
  stream << "  direction_sign: [" << ray.direction_sign[0] << "," << ray.direction_sign[1] << "," << ray.direction_sign[2] << "],\n";
  stream << "  min_hit_distance: " << ray.min_hit_distance << ",\n";
  stream << "  max_hit_distance " << ray.max_hit_distance << ",\n";
  stream << "  cull_back_face " << (ray.cull_back_face == Ray<T>::CullBackFace::no ? "no" : "yes") << ",\n";
  stream << "  any_hit: " << (ray.any_hit == Ray<T>::AnyHit::no ? "no" : "yes") << "\n";

  stream << "}\n";
  return stream;
}

template<typename T>
std::ostream &operator<<(std::ostream &stream, const RayHit<T> &rayhit) {
  /// Conveniently output a single cylinder as JSON.
  stream << "{\n";

  stream << "  Rayhit: " << &rayhit << ",\n";
  stream << "  normal: [" << rayhit.normal[0] << "," << rayhit.normal[1] << "," << rayhit.normal[2] << "],\n";
  stream << "  uv: [" << rayhit.uv[0] << "," << rayhit.uv[1] << "],\n";
  stream << "  hit_distance: "<< rayhit.hit_distance << ",\n";
  stream << "  prim_id: " << rayhit.prim_id << ",\n";
  stream << "  geom_id: " << rayhit.geom_id << ",\n";

  stream << "}\n";
  return stream;
}

}// namespace blazert

#endif// BLAZERT_RAY_H_
