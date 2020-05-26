#pragma once
#ifndef BLAZERT_BVH_AABB_H_
#define BLAZERT_BVH_AABB_H_

#include <blazert/datatypes.h>
#include <blazert/ray.h>
#include <blazert/bvh/node.h>
#include <iostream>
#include <limits>

namespace blazert {

template<typename T>
inline bool intersect_node(T &min_distance /* out */, T &max_distance /* out*/, const BVHNode<T> &node, const Ray<T> &ray) {

  constexpr T l1 = static_cast<T>(1) + static_cast<T>(4) * std::numeric_limits<T>::epsilon();
  T min_, max_;
  min_distance = ray.min_hit_distance;
  max_distance = ray.max_hit_distance;

  for (int i=0; i<3; i++) {
    min_ = ray.direction_sign[i] ? node.max[i]: node.min[i];
    min_ = (min_ - ray.origin[i]) * ray.direction_inv[i];
    min_distance = std::max(min_, min_distance);

    max_ = ray.direction_sign[i] ? node.min[i]: node.max[i];
    max_ = (max_ - ray.origin[i]) * ray.direction_inv[i] * l1;
    max_distance = std::min(max_, max_distance);
  }

  return (min_distance <= max_distance);
}
}// namespace blazert

#endif// BLAZERT_BVH_AABB_H
