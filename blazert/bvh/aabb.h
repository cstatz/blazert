#pragma once
#ifndef BLAZERT_BVH_AABB_H_
#define BLAZERT_BVH_AABB_H_

#include <blazert/datatypes.h>
#include <iostream>
#include <limits>

namespace blazert {

// TODO: Change signature to Ray<T> and BVHNode<T>. If done correctly there should not be any performance issue and testing will be a lot easier.
template<typename T>
inline bool IntersectRayAABB(T &tmin, T &tmax, const T &min_t, const T &max_t, const Vec3r<T> &bmin, const Vec3r<T> &bmax,
                             const Vec3r<T> &ray_org, const Vec3r<T> &ray_inv_dir, const Vec3ui &ray_dir_sign) {

  // TODO: This is ugly, but seems to work. Maybe not the fastest ... How can this be vectorized?
  const T min_x = ray_dir_sign[0] ? bmax[0] : bmin[0];
  const T min_y = ray_dir_sign[1] ? bmax[1] : bmin[1];
  const T min_z = ray_dir_sign[2] ? bmax[2] : bmin[2];

  const T max_x = ray_dir_sign[0] ? bmin[0] : bmax[0];
  const T max_y = ray_dir_sign[1] ? bmin[1] : bmax[1];
  const T max_z = ray_dir_sign[2] ? bmin[2] : bmax[2];

  constexpr T l1 = static_cast<T>(1) + static_cast<T>(4) * std::numeric_limits<T>::epsilon();
  const T tmin_x = (min_x - ray_org[0]) * ray_inv_dir[0];
  const T tmax_x = (max_x - ray_org[0]) * ray_inv_dir[0] * l1;

  const T tmin_y = (min_y - ray_org[1]) * ray_inv_dir[1];
  const T tmax_y = (max_y - ray_org[1]) * ray_inv_dir[1] * l1;

  const T tmin_z = (min_z - ray_org[2]) * ray_inv_dir[2];
  const T tmax_z = (max_z - ray_org[2]) * ray_inv_dir[2] * l1;

  // TODO: Benchmark this:
  const T temp_min = std::max(tmin_z, std::max(tmin_y, std::max(tmin_x, min_t)));
  const T temp_max = std::min(tmax_z, std::min(tmax_y, std::min(tmax_x, max_t)));
  //const T temp_min = std::max({tmin_z, tmin_y, tmin_x, min_t});
  //const T temp_max = std::min({tmax_z, tmax_y, tmax_x, max_t});

  // It is more likely that we hit a box.
  if (temp_min > temp_max) return false;

  tmin = temp_min;
  tmax = temp_max;

  return true;
}
}// namespace blazert
#endif// BLAZERT_BVH_AABB_H
