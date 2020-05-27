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
inline bool intersect_node(T &min_distance /* inout */, T &max_distance /* inout*/, const BVHNode<T> &node, const Ray<T> &ray) {

  constexpr T l1 = static_cast<T>(1) + static_cast<T>(4) * std::numeric_limits<T>::epsilon();
  T min_, max_;

  for (int i=0; i<3; i++) {
    min_ = ray.direction_sign[i] ? node.max[i]: node.min[i];
    max_ = ray.direction_sign[i] ? node.min[i]: node.max[i];

    min_ = (min_ - ray.origin[i]) * ray.direction_inv[i];
    max_ = (max_ - ray.origin[i]) * ray.direction_inv[i] * l1;

    min_distance = std::max(min_, min_distance);
    max_distance = std::min(max_, max_distance);
  }

  return (min_distance <= max_distance);
}
}// namespace blazert

#endif// BLAZERT_BVH_AABB_H
