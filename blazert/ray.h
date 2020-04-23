#pragma once
#ifndef BLAZERT_RAY_H_
#define BLAZERT_RAY_H_

#include <limits>

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {
// RayType
typedef enum {
  RAY_TYPE_NONE = 0x0,
  RAY_TYPE_PRIMARY = 0x1,
  RAY_TYPE_SECONDARY = 0x2,
  RAY_TYPE_DIFFUSE = 0x4,
  RAY_TYPE_REFLECTION = 0x8,
  RAY_TYPE_REFRACTION = 0x10
} RayType;

#ifdef __clang__
#pragma clang diagnostic push
#if __has_warning("-Wzero-as-null-pointer-constant")
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif
#endif

template<typename T>
class
alignas(sizeof(Vec3r<T>))
Ray {
public:
  Vec3r<T> org;
  Vec3r<T> dir;
  T min_t = 0.;                             // minimum ray hit distance.
  T max_t = std::numeric_limits<T>::max();  // maximum ray hit distance.
  unsigned int type = RAY_TYPE_NONE;        // ray type
public:
  Ray() {org = 0.; dir = {0., 0., -1.};};
  Ray(const Vec3r<T> &org_, const Vec3r<T> &dir_) : org(org_), dir(normalize(dir_)) {};
};
}// namespace blazert

#endif// BLAZERT_RAY_H_
