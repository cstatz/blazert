#ifndef BLAZERT_REAL_H_
#define BLAZERT_REAL_H_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <thread>
#include <blaze/Math.h>

using blaze::StaticVector;
using blaze::DynamicVector;
using blaze::columnVector;
using blaze::aligned;
using blaze::unaligned;
using blaze::padded;
using blaze::unpadded;


template<class T>
using real3 = StaticVector<T,3UL,columnVector,unaligned,padded>;


// ----------------------------------------------------------------------------
/**
template <typename T = float>
class real3 {
 public:
  real3() {}
  real3(T x) {
    v[0] = x;
    v[1] = x;
    v[2] = x;
  }
  real3(T xx, T yy, T zz) {
    v[0] = xx;
    v[1] = yy;
    v[2] = zz;
  }
  explicit real3(const T *p) {
    v[0] = p[0];
    v[1] = p[1];
    v[2] = p[2];
  }

  inline T x() const { return v[0]; }
  inline T y() const { return v[1]; }
  inline T z() const { return v[2]; }

  real3 operator*(T f) const { return real3(x() * f, y() * f, z() * f); }
  real3 operator-(const real3 &f2) const {
    return real3(x() - f2.x(), y() - f2.y(), z() - f2.z());
  }
  real3 operator*(const real3 &f2) const {
    return real3(x() * f2.x(), y() * f2.y(), z() * f2.z());
  }
  real3 operator+(const real3 &f2) const {
    return real3(x() + f2.x(), y() + f2.y(), z() + f2.z());
  }
  real3 &operator+=(const real3 &f2) {
    v[0] += f2.x();
    v[1] += f2.y();
    v[2] += f2.z();
    return (*this);
  }
  real3 operator/(const real3 &f2) const {
    return real3(x() / f2.x(), y() / f2.y(), z() / f2.z());
  }
  real3 operator-() const { return real3(-x(), -y(), -z()); }
  T operator[](int i) const { return v[i]; }
  T &operator[](int i) { return v[i]; }

  T v[3];
  // T pad;  // for alignment (when T = float)
};
*/

// v_functions
// Will be provided by blaze
//template <typename T>
//inline real3<T> operator*(T f, const real3<T> &v) {
//  return real3<T>(v.x() * f, v.y() * f, v.z() * f);
//}


template <typename T>
inline real3<T> vsafe_inverse(const real3<T> v) {
  // This is also handled by blaze.
  real3<T> r;
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


// Address computation
template <typename real>
inline const real *get_vertex_addr(const real *p, const size_t idx,
                                   const size_t stride_bytes) {
  return reinterpret_cast<const real *>(
      reinterpret_cast<const unsigned char *>(p) + idx * stride_bytes);
}

#endif  // BLAZERT_H_
