#pragma once
#ifndef BLAZERT_BVH_AABB_H_
#define BLAZERT_BVH_AABB_H_

#include <blazert/bvh/node.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>
#include <iostream>
#include <limits>

namespace blazert {

// min_distance and max_distance need to be initialized by the CALLER in a sensible way, because they are
// used to compare against,e.g.:
//    T min_distance = 0;
//    T max_distance = std::numeric_limit<T>::max()
template<typename T, typename Node>
inline bool intersect_node(T &min_distance /* inout */, T &max_distance /* inout*/, const Node &node,
                           const Ray<T> &ray) noexcept {

  // This is hard to beat.
  constexpr T l1 = static_cast<T>(1) + static_cast<T>(4) * std::numeric_limits<T>::epsilon();
  T min_, max_;

  for (int i = 0; i < 3; i++) {
    min_ = ray.direction_sign[i] ? node.max[i] : node.min[i];
    max_ = ray.direction_sign[i] ? node.min[i] : node.max[i];

    min_ = (min_ - ray.origin[i]) * ray.direction_inv[i];
    max_ = (max_ - ray.origin[i]) * ray.direction_inv[i] * l1;

    min_distance = std::max(min_, min_distance);
    max_distance = std::min(max_, max_distance);
  }
  return (min_distance <= max_distance);
}

template<typename Node, typename Intersector, typename Ray>
inline bool intersect_leaf(const Node &node, Intersector &intersector, const Ray &ray) noexcept {

  bool hit = false;

  for (auto &primitive : node.primitives) {
    hit += intersect_primitive(intersector, primitive, ray);
    if (hit && ray.any_hit) {
      break;
    }
  }
  return hit;
}

}// namespace blazert

#endif// BLAZERT_BVH_AABB_H
