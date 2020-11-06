//
// Created by Christoph Statz on 02.02.18.
//

#ifndef EM_ORIGINSPHERE_H
#define EM_ORIGINSPHERE_H

#include "OriginMesh.h"
#include <blazert/datatypes.h>

template<typename T>
inline void initialize_icosahedron(OriginMesh<T> &mesh) {
  const T t = (1.0f + std::sqrt(5.0f)) / 2.0f;

  // Vertices
  mesh.vertices.emplace_back(blazert::Vec3r<T>{-1.0f, t, 0.0});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{1.0, t, 0.0});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{-1.0f, -t, 0.0});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{1.0, -t, 0.0});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{0.0, -1.0f, t});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{0.0, 1.0, t});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{0.0, -1.0f, -t});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{0.0, 1.0, -t});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{t, 0.0, -1.0f});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{t, 0.0, 1.0});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{-t, 0.0, -1.0f});
  mesh.vertices.emplace_back(blazert::Vec3r<T>{-t, 0.0, 1.0});

  for (auto &it : mesh.vertices)
    it = normalize(it);

  // Faces
  mesh.add_triangle(0, 11, 5);
  mesh.add_triangle(0, 5, 1);
  mesh.add_triangle(0, 1, 7);
  mesh.add_triangle(0, 7, 10);
  mesh.add_triangle(0, 10, 11);
  mesh.add_triangle(1, 5, 9);
  mesh.add_triangle(5, 11, 4);
  mesh.add_triangle(11, 10, 2);
  mesh.add_triangle(10, 7, 6);
  mesh.add_triangle(7, 1, 8);
  mesh.add_triangle(3, 9, 4);
  mesh.add_triangle(3, 4, 2);
  mesh.add_triangle(3, 2, 6);
  mesh.add_triangle(3, 6, 8);
  mesh.add_triangle(3, 8, 9);
  mesh.add_triangle(4, 9, 5);
  mesh.add_triangle(2, 4, 11);
  mesh.add_triangle(6, 2, 10);
  mesh.add_triangle(8, 6, 7);
  mesh.add_triangle(9, 8, 1);
}

template<typename T>
inline uint32_t subdivide_edge(uint32_t f0, uint32_t f1, const blazert::Vec3r<T> &v0, const blazert::Vec3r<T> &v1,
                               OriginMesh<T> &io_mesh) {
  blazert::Vec3r<T> v = (v0 + v1) * 0.5f;
  v = normalize(v);

  const uint32_t f = io_mesh.vertices.size();
  io_mesh.vertices.emplace_back(v);
  return f;
}

template<typename T>
inline void subdivide_mesh(const OriginMesh<T> &in, OriginMesh<T> &out) {
  out.vertices = in.vertices;

  for (uint32_t i = 0; i < in.triangle_count(); ++i) {
    const uint32_t f0 = in.triangles[i][0];
    const uint32_t f1 = in.triangles[i][1];
    const uint32_t f2 = in.triangles[i][2];

    const blazert::Vec3r<T> v0 = in.vertices[f0];
    const blazert::Vec3r<T> v1 = in.vertices[f1];
    const blazert::Vec3r<T> v2 = in.vertices[f2];

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
class OriginSphere : public OriginMesh<T> {
public:
  OriginSphere() = delete;

  explicit OriginSphere(uint32_t resolution);
};

template<typename T>
OriginSphere<T>::OriginSphere(uint32_t resolution) {
  std::vector<OriginMesh<T>> meshes;

  // for (auto &m : meshes) m.clear();

  meshes.emplace_back(OriginMesh<T>());
  initialize_icosahedron(meshes.back());

  for (uint32_t i = 0; i < resolution; i++) {
    meshes.emplace_back(OriginMesh<T>());
  }

  for (uint32_t k = 0; k < resolution; ++k) {

    meshes[k + 1].reserve(20 * std::pow(4, resolution) - 8, 20 * std::pow(4, resolution) - 8);
    subdivide_mesh(meshes[k], meshes[k + 1]);
  }

  OriginMesh<T>::triangles = meshes.back().triangles;
  OriginMesh<T>::vertices = meshes.back().vertices;
}

#endif// EM_ORIGINSPHERE_H
