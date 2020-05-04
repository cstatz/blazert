//
// Created by Christoph Statz on 03.12.16.
//

#ifndef BLAZERT_PRIMITIVES_TRI_GEOMETRY_HELPER_H
#define BLAZERT_PRIMITIVES_TRI_GEOMETRY_HELPER_H
#include <blazert/datatypes.h>

namespace blazert {

/**
   The algorithm is based on:
   David Eberly: "Distance Between Point and Triangle in 3D", Geometric Tools, LLC (1999)
   http:\\www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf

          ^ hit_distance
    \     |
     \  2 |
      \   |
       \  |
        \ |
         \|
          *(v2)
          |\
          | \
       3  |  \    1
          |   \
          |  0 \
          |     \
          |      \ (v1)
   -------*-------*-------> s
          |(v0)    \
       4  |    5    \    6
*/

template<typename T>
inline void r0(T &s, T &t, const T &det) {
  const T inv_det = T(1.) / det;
  s *= inv_det;
  t *= inv_det;
};

template<typename T>
inline void r1(T &s, T &t, const T &a, const T &b, const T &c, const T &d, const T &e) {
  const T numer = c + e - b - d;
  if (numer <= T(0.)) {
    s = T(0.);
  } else {
    const T denom = a - T(2.)*b + c;
    if (numer >= denom) {
      s = T(1.);
    } else {
      s = numer/denom;
    }
  }
  t = T(1.) - s;
};

template<typename T>
inline void r2(T &s, T &t, const T &a, const T &b, const T &c, const T &d, const T &e) {
  const T tmp0 = b + d;
  const T tmp1 = c + e;
  if (tmp1 > tmp0) {
    const T numer = tmp1 - tmp0;
    const T denom = a - T(2.)*b + c;
    if (numer >= denom) {
      s = T(1.);
    } else {
      s = numer / denom;
    }
    t = T(1.) - s;
  } else {
    s = T(0.);
    if (tmp1 <= T(0.))
      t = T(1.);
    else {
      if (e >= T(0.))
        t = T(0.);
      else
        t = -e/c;
    }
  }
};

template<typename T>
inline void r3(T &s, T &t, const T &c, const T &e) {

  s = T(0.);

  if (e >= T(0.)) {
    t = T(0.);
  } else {
    if (-e >= c) {
      t = T(1.);
    } else {
      t = -e/c;
    }
  }
};

template<typename T>
inline void r4(T &s, T &t, const T &a, const T &c, const T &d, const T &e) {
  if (d < T(0.)) {
    t = T(0.);
    if (-d >= a) {
      s = T(1.);
    } else {
      s = -d/a;
    }

  } else {
    s = T(0.);
    if (e >= T(0))
      t = T(0.);
    else {
      if (-e >= c)
        t = T(1.);
      else
        t = -e/c;
    }
  }
};

template<typename T>
inline void r5(T &s, T &t, const T &a, const T &d, const T &f) {

  t = T(0.);

  if (d >= T(0.)) {
    s = T(0.);
  } else {
    if (-d >= a) {
      s = T(1.);
    } else {
      s = -d/a;
    }
  }
};

template<typename T>
inline void r6(T &s, T &t, const T &a, const T &b, const T &c, const T &d, const T &e) {
  const T tmp0 = b + e;
  const T tmp1 = a + d;
  if (tmp1 > tmp0) {
    const T numer = tmp1 - tmp0;
    const T denom = a - T(2.)*b + c;
    if (numer >= denom) {
      t = T(1.);
    } else {
      t = numer / denom;
    }
    s = T(1.) - t;
  } else {
    t = T(0.);
    if (tmp1 <= T(0.))
      s = T(1.);
    else {
      if (d >= T(0.))
        s = T(0.);
      else
        s = -d/a;
    }
  }
};

/**
 * @brief Compute the closest point on a triangle for a given test point. This only works for well-formed triangles.
 *
 * @tparam T any real type for which a vector class is implemented that provides a dot function.
 * @param tri_v0 3-Vector of type T representing triangle vertex 0.
 * @param tri_v1 3-Vector of type T representing triangle vertex 1.
 * @param tri_v2 3-Vector of type T representing triangle vertex 2.
 * @param test_p 3-Vector of type T representing the test point.
 * @return 3-Vector of type T representing the closest point on the triangle.
 */
template<typename T>
inline Vec3r<T> closest_point_on_triangle(const Vec3r<T> &tri_v0, const Vec3r<T> &tri_v1, const Vec3r<T> &tri_v2, const Vec3r<T> &test_p) {

  const Vec3r<T> E0 = tri_v1 - tri_v0;
  const Vec3r<T> E1 = tri_v2 - tri_v0;
  const Vec3r<T> D = tri_v0 - test_p;

  const T a = dot(E0, E0);
  const T b = dot(E0, E1);      // Ideally close to 0 (edges 0 and 1 are orthogonal)
  const T c = dot(E1, E1);
  const T d = dot(E0, D);
  const T e = dot(E1, D);
  const T f = dot(D, D);
  const T det = a*c - b*b;  // only gives good results for well-formed triangles.
  T s = b*e - c*d;
  T t = b*d - a*e;

  if (s + t <= det) {
    if (s < T(0)) {
      if (t < T(0)) {     r4(s, t, a, c, d, e);     }
      else {              r3(s, t, c, e);           }
    }
    else if (t < T(0)) {  r5(s, t, a, d, f);        }
    else {                r0(s, t, det);            }
  }
  else {
    if (s < T(0)) {       r2(s, t, a, b, c, d, e);  }
    else if (t < T(0)) {  r6(s, t, a, b, c, d, e);  }
    else {                r1(s, t, a, b, c, d, e);  }
  }

  return tri_v0 + s*E0 + t*E1;
};

}// namespace blazert

#endif//BLAZERT_PRIMITIVES_TRI_GEOMETRY_HELPER_H
