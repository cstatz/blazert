#pragma once
#ifndef BLAZERT_BVH_AABB_H_
#define BLAZERT_BVH_AABB_H_

#include <blazert/datatypes.h>
#include <limits>

namespace blazert {

// NaN-safe min and max function.
template<class T>
const T &safemin(const T &a, const T &b) {
  return (a < b) ? a : b;
}
template<class T>
const T &safemax(const T &a, const T &b) {
  return (a > b) ? a : b;
}

template<typename T>
inline bool IntersectRayAABB(T *tminOut,// [out]
                             T *tmaxOut,// [out]
                             T min_t, T max_t, const Vec3r<T> &bmin, const Vec3r<T> &bmax,
                             Vec3r<T> &ray_org, Vec3r<T> &ray_inv_dir,
                             Vec3i ray_dir_sign) {

  T tmin, tmax;

  const T min_x = ray_dir_sign[0] ? bmax[0] : bmin[0];
  const T min_y = ray_dir_sign[1] ? bmax[1] : bmin[1];
  const T min_z = ray_dir_sign[2] ? bmax[2] : bmin[2];
  const T max_x = ray_dir_sign[0] ? bmin[0] : bmax[0];
  const T max_y = ray_dir_sign[1] ? bmin[1] : bmax[1];
  const T max_z = ray_dir_sign[2] ? bmin[2] : bmax[2];

  // X
  const T tmin_x = (min_x - ray_org[0]) * ray_inv_dir[0];
  // MaxMult robust BVH traversal(up to 4 ulp).
  const T tmax_x =
      (max_x - ray_org[0]) * ray_inv_dir[0] * (1 + 4 * std::numeric_limits<T>::epsilon());

  // Y
  const T tmin_y = (min_y - ray_org[1]) * ray_inv_dir[1];
  const T tmax_y =
      (max_y - ray_org[1]) * ray_inv_dir[1] * (1 + 4 * std::numeric_limits<T>::epsilon());

  // Z
  const T tmin_z = (min_z - ray_org[2]) * ray_inv_dir[2];
  const T tmax_z =
      (max_z - ray_org[2]) * ray_inv_dir[2] * (1 + 4 * std::numeric_limits<T>::epsilon());

  tmin = safemax(tmin_z, safemax(tmin_y, safemax(tmin_x, min_t)));
  tmax = safemin(tmax_z, safemin(tmax_y, safemin(tmax_x, max_t)));

  if (tmin <= tmax) {
    (*tminOut) = tmin;
    (*tmaxOut) = tmax;

    return true;
  }
  return false;// no hit
}
}// namespace blazert
#endif// BLAZERT_BVH_AABB_H
