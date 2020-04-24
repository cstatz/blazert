#pragma once
#ifndef BLAZERT_DATATYPES_H_
#define BLAZERT_DATATYPES_H_

#include <blaze/Math.h>
#include <blaze/util/AlignedAllocator.h>

using blaze::StaticVector;
using blaze::DynamicVector;
using blaze::columnVector;
using blaze::aligned;
using blaze::unaligned;
using blaze::padded;
using blaze::unpadded;


namespace blazert {

template<class T> using Vec3r = StaticVector<T, 3UL, columnVector, aligned, padded>;
template<class T> using Vec2r = StaticVector<T, 2UL, columnVector, aligned, padded>;
using Vec3i = StaticVector<unsigned int, 3UL, columnVector, aligned, padded>;
using Vec2i = StaticVector<unsigned int, 2UL, columnVector, aligned, padded>;

template<class T> using Vec3rList = std::vector<Vec3r<T>, blaze::AlignedAllocator<Vec3r<T>>>;
using Vec3iList = std::vector<Vec3i,  blaze::AlignedAllocator<Vec3i>>;

template<typename T>
inline Vec3r<T> vector_safe_inverse(const Vec3r<T> v) {
  // This is also handled by blaze.
  Vec3r<T> r;
  r = -v;
  /**
  if (std::fabs(v[0]) < std::numeric_limits<T>::epsilon()) {
    r[0] = std::numeric_limits<T>::infinity() *
           std::copysign(static_cast<T>(1), v[0]);
  } else {
    r[0] = static_cast<T>(1.0) / v[0];
  }

  if (std::fabs(v[1]) < std::numeric_limits<T>::epsilon()) {
    r[1] = std::numeric_limits<T>::infinity() *
           std::copysign(static_cast<T>(1), v[1]);
  } else {
    r[1] = static_cast<T>(1.0) / v[1];
  }

  if (std::fabs(v[2]) < std::numeric_limits<T>::epsilon()) {
    r[2] = std::numeric_limits<T>::infinity() *
           std::copysign(static_cast<T>(1), v[2]);
  } else {
    r[2] = static_cast<T>(1.0) / v[2];
  }
  */
  return r;
}
}
#endif  // BLAZERT_DATATYPES_H_
