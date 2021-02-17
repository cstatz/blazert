//
// Created by ogarten on 1/23/19.
// Modified Christoph Statz on 27.04.20.
//

#ifndef BLAZERT_TEST_TEST_HELPERS_H
#define BLAZERT_TEST_TEST_HELPERS_H
//
// This header includes all relevant headers which are needed for all test
// cases. It also provides general functions that could be needed for testing.
//

#include "pmmintrin.h"
#include "xmmintrin.h"
#include <blazert/datatypes.h>

using namespace std::literals::complex_literals;
using namespace blazert;

template<typename T>
constexpr T pi = static_cast<T>(3.141592653589793238462643383279502884197);

static inline bool is_aligned(const void *ptr, size_t byte_count) {
  return reinterpret_cast<uintptr_t>(ptr) % byte_count == 0;
}

template<typename T>
inline bool is_almost_equal(const T &in_a, const T &in_b, const double eps) {
  return (std::abs(in_a - in_b) <= eps);
}

/**
 * Creates a triangle mesh of a cube with counterclockwise vertex count and saves the vertices and indices in the corresponding
 * function arguments.
 *
 * The bounding box is for center = (0,0,0) is defined by the box spanned from (-1,-1,-1) to (1,1,1).
 *
 * @tparam T floating point type
 * @param center center of the cube
 * @param vertices Vec3rList<T> with the vertices of the cube
 * @param indices Vec3iList<T> with the indices
 */
template<typename T>
inline void cube_mesh_ccw(const Vec3r<T> &center, Vec3rList<T> &vertices, Vec3iList &indices) {
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(-1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(-1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(-1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(-1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(1.), center[2] + T(1.)});

  // CCW Tris
  indices.emplace_back(Vec3ui{1, 5, 4});
  indices.emplace_back(Vec3ui{0, 1, 4});
  indices.emplace_back(Vec3ui{2, 6, 5});
  indices.emplace_back(Vec3ui{1, 2, 5});
  indices.emplace_back(Vec3ui{3, 7, 6});
  indices.emplace_back(Vec3ui{2, 3, 6});
  indices.emplace_back(Vec3ui{4, 7, 3});
  indices.emplace_back(Vec3ui{0, 4, 3});
  indices.emplace_back(Vec3ui{5, 6, 7});
  indices.emplace_back(Vec3ui{4, 5, 7});
  indices.emplace_back(Vec3ui{3, 2, 1});
  indices.emplace_back(Vec3ui{0, 3, 1});
}
/**
 * Creates a triangle mesh of a cube with clockwise vertex count and saves the vertices and indices in the corresponding
 * function arguments.
 *
 * The bounding box is for center = (0,0,0) is defined by the box spanned from (-1,-1,-1) to (1,1,1).
 *
 * @tparam T floating point type
 * @param center center of the cube
 * @param vertices Vec3rList<T> with the vertices of the cube
 * @param indices Vec3iList<T> with the indices
 */
template<typename T>
inline void cube_mesh_cw(const Vec3r<T> &center, Vec3rList<T> &vertices, Vec3iList &indices) {
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(-1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(-1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(-1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(-1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(1.), center[2] + T(1.)});

  // CW Tris
  indices.emplace_back(Vec3ui{1, 4, 5});
  indices.emplace_back(Vec3ui{0, 4, 1});
  indices.emplace_back(Vec3ui{2, 5, 6});
  indices.emplace_back(Vec3ui{1, 5, 2});
  indices.emplace_back(Vec3ui{3, 6, 7});
  indices.emplace_back(Vec3ui{2, 6, 3});
  indices.emplace_back(Vec3ui{4, 3, 7});
  indices.emplace_back(Vec3ui{0, 3, 4});
  indices.emplace_back(Vec3ui{5, 7, 6});
  indices.emplace_back(Vec3ui{4, 7, 5});
  indices.emplace_back(Vec3ui{3, 1, 2});
  indices.emplace_back(Vec3ui{0, 1, 3});
}

/**
 * Creates a single triangle with counterclockwise vertex count and saves the vertices and indices in the corresponding
 * function arguments.
 *
 * The bounding box is for center = (0,0,0) is defined by the box spanned from (-1,-1,-1) to (1,1,1).
 *
 * @tparam T floating point type
 * @param center center of the triangle
 * @param vertices Vec3rList<T> with the vertices of the triangle
 * @param indices Vec3iList<T> with the indices
 */
template<typename T>
inline void single_triangle_ccw(const Vec3r<T> &center, Vec3rList<T> &vertices, Vec3iList &indices) {
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(-1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(1.), center[2] + T(-1.)});

  indices.emplace_back(Vec3ui{0, 2, 1});
}

/**
 * Creates a single triangle with clockwise vertex count and saves the vertices and indices in the corresponding
 * function arguments.
 *
 * The bounding box is for center = (0,0,0) is defined by the box spanned from (-1,-1,-1) to (1,1,1).
 *
 * @tparam T floating point type
 * @param center center of the triangle
 * @param vertices Vec3rList<T> with the vertices of the triangle
 * @param indices Vec3iList<T> with the indices
 */
template<typename T>
inline void single_triangle_cw(const Vec3r<T> &center, Vec3rList<T> &vertices, Vec3iList &indices) {
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(-1.), center[2] + T(-1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(-1.), center[1] + T(1.), center[2] + T(1.)});
  vertices.emplace_back(Vec3r<T>{center[0] + T(1.), center[1] + T(1.), center[2] + T(-1.)});

  indices.emplace_back(Vec3ui{0, 1, 2});
}

/**
 * Returns true if all elements of m1 and m2 are approximately equal
 * @tparam T1 floating point type of m1
 * @tparam T2 floating point type of m2
 * @param m1 matrix 1
 * @param m2 matrix 2
 * @return true, if approx. equal
 */
template<typename T1, typename T2>
bool Mat3_isApprox(Mat3r<T1> &m1, Mat3r<T2> &m2) {
  bool res = true;
  for (size_t i = 0; i <= 2; ++i) {
    for (size_t j = 0; j <= 2; ++j) {
      res = res && m1(i, j) == Approx(m2(i, j));
    }
  }
  return res;
}

/***
 * arbitraryRotationMatrix takes an axis which the rotation is about and an
 * angle which specifies how far the rotation is made. The definition is given
 * here:
 * \url{https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle}
 * It is assumed that the rotation axis starts at the origin.
 * If the 3D space is right-handed, the rotation will be counter clockwise in
 * regard to the axis (right hand rule)
 *
 * @param axis normalize vector giving the rotation axis
 * @param angle rotation angle around axis
 * @return
 */
template<typename T>
inline Mat3r<T> arbitraryRotationMatrix(const Vec3r<T> &axis, const T angle) {
  const T n1 = axis[0];
  const T n2 = axis[1];
  const T n3 = axis[2];

  const T sina = std::sin(angle);
  const T cosa = std::cos(angle);

  const Mat3r<T> rot{{n1 * n1 * (1 - cosa) + cosa, n1 * n2 * (1 - cosa) - n3 * sina, n1 * n3 * (1 - cosa) + n2 * sina},
                     {n1 * n2 * (1 - cosa) + n3 * sina, n2 * n2 * (1 - cosa) + cosa, n2 * n3 * (1 - cosa) - n1 * sina},
                     {n1 * n3 * (1 - cosa) - n2 * sina, n2 * n3 * (1 - cosa) + n1 * sina, n3 * n3 * (1 - cosa) + cosa}};
  return rot;
}

#endif//BLAZERT_TEST_TEST_HELPERS_H
