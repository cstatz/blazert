#pragma once
#ifndef BLAZERT_BVH_AABB_H_
#define BLAZERT_BVH_AABB_H_

#include <blazert/datatypes.h>
#include <iostream>
#include <limits>

namespace blazert {

// NaN-safe min and max function.
template <class T>
const T &safemin(const T &a, const T &b) {
  return (a < b) ? a : b;
}
template <class T>
const T &safemax(const T &a, const T &b) {
  return (a > b) ? a : b;
}

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

  // X
  const T tmin_x = (min_x - ray_org[0]) * ray_inv_dir[0];
  const T tmax_x = (max_x - ray_org[0]) * ray_inv_dir[0] * (1 + 4 * std::numeric_limits<T>::epsilon());

  // Y
  const T tmin_y = (min_y - ray_org[1]) * ray_inv_dir[1];
  const T tmax_y = (max_y - ray_org[1]) * ray_inv_dir[1] * (1 + 4 * std::numeric_limits<T>::epsilon());

//  if ((tmin_x > tmax_y) || (tmin_y > tmax_x)) {
//    return false;
//  }
//
//  if (tmin_y > tmin_x)
//    tmin_x = tmin_y;
  //if (tmax_y < tmax_x)
  //  tmax_x = tmax_y;


  const T tmin_z = (min_z - ray_org[2]) * ray_inv_dir[2];
  const T tmax_z = (max_z - ray_org[2]) * ray_inv_dir[2] * (1 + 4 * std::numeric_limits<T>::epsilon());

  const T temp_min = safemax(tmin_z, safemax(tmin_y, safemax(tmin_x, min_t)));
  const T temp_max = safemin(tmax_z, safemin(tmax_y, safemin(tmax_x, max_t)));

  //if ((tmin_x > tmax_z) || (tmin_z > tmax_x)) {
  //  return false;
  //}
//  if (tmin_z > tmin_x)
//    tmin_x = tmin_z;
//  if (tmax_z < tmax_x)
//    tmax_x = tmax_z;

//  if ((tmin_x >= min_t) && (tmax_x < max_t)) {
//    tmin = tmin_x;
//    tmax = tmax_x;
//    return true;
//  }
  if (temp_min <= temp_max) {
    tmin = temp_min;
    tmax = temp_max;

    return true;
  }

  return false;
}
}// namespace blazert
#endif// BLAZERT_BVH_AABB_H
