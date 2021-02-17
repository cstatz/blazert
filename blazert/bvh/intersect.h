#pragma once
#ifndef BLAZERT_BVH_AABB_H_
#define BLAZERT_BVH_AABB_H_

#include <blazert/bvh/node.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>
#include <iostream>
#include <limits>

namespace blazert {


/**
 * Tests whether a ray intersects with a node of the BVH.
 *
 * min_distance and max_distance need to be initialized by the CALLER in a sensible way, because they are
 * used to compare against the hit distance,e.g.:
 * T min_distance = 0;
 * T max_distance = std::numeric_limit<T>::max()
 *
 * Valid hits are in the interval [min_distance; max_distance]
 *
 * @tparam T floating point type
 * @tparam Node Node type
 * @param min_distance  minimum hit distance which is considered
 * @param max_distance maximum hit distance which is considered
 * @param node node of the acceleration structure to test
 * @param ray ray which is tested
 * @return returns true if node it hit and hit distance is in interval [min_distance; max_distance]
 */
template<typename T, typename Node>
inline bool intersect_node(T &min_distance /* inout */, T &max_distance /* inout*/, const Node &node,
                           const Ray<T> &ray) noexcept {

  // This is hard to beat.
  constexpr T l1 = static_cast<T>(1) + static_cast<T>(4) * std::numeric_limits<T>::epsilon();
  T min_, max_;

  for (std::size_t i = 0; i < 3; i++) {
    min_ = ray.direction_sign[i] ? node.max[i] : node.min[i];
    max_ = ray.direction_sign[i] ? node.min[i] : node.max[i];

    min_ = (min_ - ray.origin[i]) * ray.direction_inv[i];
    max_ = (max_ - ray.origin[i]) * ray.direction_inv[i] * l1;

    min_distance = std::max(min_, min_distance);
    max_distance = std::min(max_, max_distance);
  }
  return (min_distance <= max_distance);
}

/**
 * Tests whether any primitive in the leaf node `node` is hit by the ray.
 *
 * The Intersector is needed to calculate the actual intersection of the ray with the primitive.
 *
 * @tparam Node Node type
 * @tparam Intersector Intersector type
 * @tparam Ray type
 * @param node node containing the primitives (needs to be a leaf node)
 * @param intersector intersector of the primitives which also saves the intersection distance etc.
 * @param ray ray for which the intersection is calculated
 * @return true, if the ray hits any primitive;
 */
template<typename Node, typename Intersector, typename Ray>
inline bool intersect_leaf(const Node &node, Intersector &intersector, const Ray &ray) noexcept {

  bool hit = false;

  for (auto &primitive : node.primitives) {
    hit += intersect_primitive(intersector, primitive, ray);
    if (hit && (ray.any_hit == Ray::AnyHit::yes)) {
      break;
    }
  }
  return hit;
}

}// namespace blazert

#endif// BLAZERT_BVH_AABB_H
