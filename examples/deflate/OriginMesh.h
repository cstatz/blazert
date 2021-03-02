//
// Created by Christoph Statz on 04.06.18.
//

#ifndef EM_ORIGINMESH_H
#define EM_ORIGINMESH_H

#include <blazert/blazert.h>
#include "constants.h"

using namespace blazert;
using namespace emrt::constants;

namespace emrt::origin {

template<typename T>  // GeometryType
class Mesh
{
public:
  Mesh() = default;
  virtual ~Mesh() = default;

  std::vector<blazert::Vec3r<T>> vertices;
  std::vector<uint32_t> triangles;

  uint32_t triangle_count() const {return triangles.size() / 3;};
  uint32_t vertices_count() const {return vertices.size();};
  void add_triangle(uint32_t a, uint32_t b, uint32_t c)
  {
    triangles.emplace_back(a); // 1
    triangles.emplace_back(b); // 2
    triangles.emplace_back(c); // 3
  };
  void clear()
  {
    vertices.clear();
    triangles.clear();
  };
  void reserve(uint32_t t, uint32_t f)
  {
    vertices.reserve(t);
    triangles.reserve(f * 3);
  }
};

template<typename T>
inline uint32_t subdivide_edge(uint32_t f0, uint32_t f1, const Vec3r<T>& v0, const Vec3r<T>& v1, Mesh<T>& io_mesh)
{
  Vec3r<T> v = (v0 + v1) * static_cast<T>(0.5);
  v = normalize(v);

  const uint32_t f = io_mesh.vertices.size();
  io_mesh.vertices.emplace_back(v);
  return f;
}

template<typename T>
inline void subdivide_mesh(const Mesh<T>& in, Mesh<T>& out)
{
  out.vertices = in.vertices;

  for (uint32_t i = 0; i < in.triangle_count(); ++i) {
    const uint32_t f0 = in.triangles[i * 3 + 0];
    const uint32_t f1 = in.triangles[i * 3 + 1];
    const uint32_t f2 = in.triangles[i * 3 + 2];

    const Vec3r<T> v0 = in.vertices[f0];
    const Vec3r<T> v1 = in.vertices[f1];
    const Vec3r<T> v2 = in.vertices[f2];

    const uint32_t f3 = subdivide_edge(f0, f1, v0, v1, out);
    const uint32_t f4 = subdivide_edge(f1, f2, v1, v2, out);
    const uint32_t f5 = subdivide_edge(f2, f0, v2, v0, out);

    out.add_triangle(f0, f3, f5);
    out.add_triangle(f3, f1, f4);
    out.add_triangle(f4, f2, f5);
    out.add_triangle(f3, f4, f5);
  }
}

template<typename T>
inline void subdivide_mesh(const Mesh<T>& in, Mesh<T>& out, const blazert::Vec3r<T> &dir, const double angle_cut)
{
  out.vertices = in.vertices;

  for (uint32_t i = 0; i < in.triangle_count(); ++i) {
    const uint32_t f0 = in.triangles[i * 3 + 0];
    const uint32_t f1 = in.triangles[i * 3 + 1];
    const uint32_t f2 = in.triangles[i * 3 + 2];

    const blazert::Vec3r<T> &v0 = in.vertices[f0];
    const blazert::Vec3r<T> &v1 = in.vertices[f1];
    const blazert::Vec3r<T> &v2 = in.vertices[f2];

    // if dir != 0,0,0 and angle smaller then certain angle
    const double norm_dir = norm(dir);

    const double norm_0 = norm(v0);
    const double scalar_prod_0 = dot(v0, dir);
    const double angle_0 = acos(scalar_prod_0 / (norm_0 * norm_dir));

    const double norm_1 = norm(v1);
    const double scalar_prod_1 = dot(v1, dir);
    const double angle_1 = acos(scalar_prod_1 / (norm_1 * norm_dir));

    const double norm_2 = norm(v2);
    const double scalar_prod_2 = dot(v2, dir);
    const double angle_2 = acos(scalar_prod_2 / (norm_2 * norm_dir));

    // std::cout << "angle = " << angle*rad2deg << std::endl;
    if ((angle_0 < angle_cut / static_cast<T>(2) * deg2rad<T>) || (angle_1 < angle_cut / static_cast<T>(2) * deg2rad<T>) ||
      (angle_2 < angle_cut / static_cast<T>(2) * deg2rad<T>)) {
      // std::cout << "angle = " << angle*rad2deg << std::endl;
      const uint32_t f3 = subdivide_edge(f0, f1, v0, v1, out);
      const uint32_t f4 = subdivide_edge(f1, f2, v1, v2, out);
      const uint32_t f5 = subdivide_edge(f2, f0, v2, v0, out);

      out.add_triangle(f0, f3, f5);
      out.add_triangle(f3, f1, f4);
      out.add_triangle(f4, f2, f5);
      out.add_triangle(f3, f4, f5);
    }
  }
}


}
#endif // EM_ORIGINMESH_H
