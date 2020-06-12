#pragma once
#ifndef BLAZERT_DATATYPES_H_
#define BLAZERT_DATATYPES_H_

#include <cmath>
#include <cstring>//for memcpy
//#define FP_FAST_FMA

#include <blaze/Math.h>
#include <blaze/util/AlignedAllocator.h>
#include <iostream>

#ifdef EMBREE_TRACING
// Embree will not work if the data is not at least padded.
#define BLAZERT_USE_PADDED_AND_ALIGNED_TYPES
#endif

#ifndef BLAZERT_USE_PADDED_AND_ALIGNED_TYPES
#define P_ blaze::unpadded
#define A_ blaze::unaligned
#define BLAZERTALIGN
#else
#define P_ padded
#define A_ aligned
#define BLAZERTALIGN alignas(sizeof(Vec3r<T>))
#endif

// You can redefine these types according to you needs by by including the definitions prior to any blazert header and defining BLAZERT_DATATYPES_H_
namespace blazert {

// Vectors
template<class T>
using Vec3r = blaze::StaticVector<T, 3UL, blaze::columnVector, A_, P_>;
template<class T>
using Vec2r = blaze::StaticVector<T, 2UL, blaze::columnVector, A_, P_>;
using Vec3ui = blaze::StaticVector<unsigned int, 3UL, blaze::columnVector, A_, P_>;
using vec2ui = blaze::StaticVector<unsigned int, 2UL, blaze::columnVector, A_, P_>;

// Matrices
template<class T>
using Mat3r = blaze::StaticMatrix<T, 3UL, 3UL, blaze::rowMajor, A_, P_>;

// Container
template<class T>
using Vec3rList = std::vector<Vec3r<T>, blaze::AlignedAllocator<Vec3r<T>>>;
using Vec3iList = std::vector<Vec3ui, blaze::AlignedAllocator<Vec3ui>>;
template<typename T>
using Mat3rList = std::vector<Mat3r<T>, blaze::AlignedAllocator<Mat3r<T>>>;

template<typename T, unsigned int capacity = 0>
struct Stack {
  T stack[capacity];
  unsigned int ss = 0;
  inline void push_back(T node) { stack[ss++] = node; }
  inline void pop_back() { stack[ss--] = 0; }
  [[nodiscard]] inline T back() const { return stack[ss - 1]; }
  [[nodiscard]] inline unsigned int size() const { return ss; }
};

template<typename T, typename F>
T as(F from) {
  static_assert(sizeof(T) == sizeof(F));
  T to;
  std::memcpy(&to, &from, sizeof(from));
  return to;
}

inline float product_sign(float x, float y) {
  return as<float>(as<uint32_t>(x) ^ (as<uint32_t>(y) & UINT32_C(0x80000000)));
}

inline double product_sign(double x, double y) {
  return as<double>(as<uint64_t>(x) ^ (as<uint64_t>(y) & UINT64_C(0x8000000000000000)));
}

template<typename T>
inline void unity(Vec3r<T> &min_, Vec3r<T> &max_, const Vec3r<T> &min, const Vec3r<T> &max) {
  for (unsigned int k = 0; k < 3; k++) {
    min_[k] = std::min(min[k], min_[k]);
    max_[k] = std::max(max[k], max_[k]);
  }
}

template<typename T>
inline void intersection(Vec3r<T> &min_, Vec3r<T> &max_, const Vec3r<T> &min, const Vec3r<T> &max) {
  min_ = blaze::min(min, min_);
  max_ = blaze::max(max, max_);
  //  for (unsigned int k = 0; k < 3; k++) {
  //    min_[k] = std::max(min[k], min_[k]);
  //    max_[k] = std::min(max[k], max_[k]);
  //  }
}

}// namespace blazert
#endif// BLAZERT_DATATYPES_H_
