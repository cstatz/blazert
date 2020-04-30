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
#include <blazert/blazert.h>

using namespace std::literals::complex_literals;
using namespace blazert;

static inline bool
is_aligned(const void* ptr, size_t byte_count)
{
  return (uintptr_t)ptr % byte_count == 0;
}

template<typename T>
inline bool
is_almost_equal(const T& in_a, const T& in_b, const double eps)
{
  return (std::abs(in_a - in_b) <= eps);
}

template<typename T>
inline void cube_mesh_ccw(const Vec3r<T>& center, Vec3rList<T>& vertices, Vec3iList& indices)
{
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T(-1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T(-1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T(-1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T(-1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T( 1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T( 1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T( 1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T( 1.), center[2] + T( 1.) });

  // CCW Tris
  indices.emplace_back(Vec3ui{ 1, 5, 4 });
  indices.emplace_back(Vec3ui{ 0, 1, 4 });
  indices.emplace_back(Vec3ui{ 2, 6, 5 });
  indices.emplace_back(Vec3ui{ 1, 2, 5 });
  indices.emplace_back(Vec3ui{ 3, 7, 6 });
  indices.emplace_back(Vec3ui{ 2, 3, 6 });
  indices.emplace_back(Vec3ui{ 4, 7, 3 });
  indices.emplace_back(Vec3ui{ 0, 4, 3 });
  indices.emplace_back(Vec3ui{ 5, 6, 7 });
  indices.emplace_back(Vec3ui{ 4, 5, 7 });
  indices.emplace_back(Vec3ui{ 3, 2, 1 });
  indices.emplace_back(Vec3ui{ 0, 3, 1 });
};

template<typename T>
inline void cube_mesh_cw(const Vec3r<T>& center, Vec3rList<T>& vertices, Vec3iList& indices)
{
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T(-1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T(-1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T(-1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T(-1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T( 1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T( 1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T( 1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T( 1.), center[2] + T( 1.) });

  // CW Tris
  indices.emplace_back(Vec3ui{ 1, 4, 5 });
  indices.emplace_back(Vec3ui{ 0, 4, 1 });
  indices.emplace_back(Vec3ui{ 2, 5, 6 });
  indices.emplace_back(Vec3ui{ 1, 5, 2 });
  indices.emplace_back(Vec3ui{ 3, 6, 7 });
  indices.emplace_back(Vec3ui{ 2, 6, 3 });
  indices.emplace_back(Vec3ui{ 4, 3, 7 });
  indices.emplace_back(Vec3ui{ 0, 3, 4 });
  indices.emplace_back(Vec3ui{ 5, 7, 6 });
  indices.emplace_back(Vec3ui{ 4, 7, 5 });
  indices.emplace_back(Vec3ui{ 3, 1, 2 });
  indices.emplace_back(Vec3ui{ 0, 1, 3 });
};

template<typename T>
inline void single_triangle_ccw(const Vec3r<T>& center, Vec3rList<T>& vertices, Vec3iList& indices)
{
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T(-1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T( 1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T( 1.), center[2] + T(-1.) });

  indices.emplace_back(Vec3ui{ 0, 2, 1 });
};

template<typename T>
inline void single_triangle_cw(const Vec3r<T>& center, Vec3rList<T>& vertices, Vec3iList& indices)
{
  // BBox (-1, -1, -1) ( 1,  1,  1)
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T(-1.), center[2] + T(-1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T(-1.), center[1] + T( 1.), center[2] + T( 1.) });
  vertices.emplace_back(Vec3r<T>{ center[0] + T( 1.), center[1] + T( 1.), center[2] + T(-1.) });

  indices.emplace_back(Vec3ui{ 0, 1, 2 });
};

template<typename T1, typename T2>
bool Mat3_isApprox(Mat3r<T1> &m1, Mat3r<T2> &m2){
  bool res = true;
  for (size_t i = 0; i <= 2; ++i)
  {
    for (size_t j = 0; j <= 2; ++j)
    {
      res = res && m1(i,j) == Approx(m2(i,j));
    }
  }
  return res;
}

#endif//BLAZERT_TEST_TEST_HELPERS_H
