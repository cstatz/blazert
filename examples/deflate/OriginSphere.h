//
// Created by Christoph Statz on 02.02.18.
//

#ifndef EM_ORIGINSPHERE_H
#define EM_ORIGINSPHERE_H

#include "OriginMesh.h"

namespace emrt::origin {

template<typename T>
inline void initialize_icosahedron(Mesh<T>& mesh)
{

  const auto t = (T{1.0} + std::sqrt(T{5.0})) / T{2.0};
  const auto o = T{1.0};
  const auto m = T{-1.0};
  const auto n = T{0.0};

  // Vertices
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  m,  t,  n });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  o,  t,  n });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  m, -t,  n });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  o, -t,  n });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  n,  m,  t });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  n,  o,  t });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  n,  m, -t });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  n,  o, -t });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  t,  n,  m });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{  t,  n,  o });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{ -t,  n,  m });
  mesh.vertices.emplace_back(blazert::Vec3r<T>{ -t,  n,  o });

  for (auto& it : mesh.vertices)
    it = normalize(it);

  // Faces
  mesh.add_triangle( 0, 11,  5);
  mesh.add_triangle( 0,  5,  1);
  mesh.add_triangle( 0,  1,  7);
  mesh.add_triangle( 0,  7, 10);
  mesh.add_triangle( 0, 10, 11);
  mesh.add_triangle( 1,  5,  9);
  mesh.add_triangle( 5, 11,  4);
  mesh.add_triangle(11, 10,  2);
  mesh.add_triangle(10,  7,  6);
  mesh.add_triangle( 7,  1,  8);
  mesh.add_triangle( 3,  9,  4);
  mesh.add_triangle( 3,  4,  2);
  mesh.add_triangle( 3,  2,  6);
  mesh.add_triangle( 3,  6,  8);
  mesh.add_triangle( 3,  8,  9);
  mesh.add_triangle( 4,  9,  5);
  mesh.add_triangle( 2,  4, 11);
  mesh.add_triangle( 6,  2, 10);
  mesh.add_triangle( 8,  6,  7);
  mesh.add_triangle( 9,  8,  1);
}


template<typename T>
class Sphere : public Mesh<T>
{
public:
  Sphere() = delete;

  explicit Sphere(uint32_t resolution, const blazert::Vec3r<T> &origin, const double radius)
  {
    std::vector<Mesh<T>> meshes;

    // for (auto &m : meshes) m.clear();

    meshes.emplace_back(Mesh<T>());
    initialize_icosahedron(meshes.back());

    for (uint32_t i = 0; i < resolution; i++) {
      meshes.emplace_back(Mesh<T>());
    }

    for (uint32_t k = 0; k < resolution; ++k) {
      try {
        // meshes[k+1].reserve(20*(4^(k+1))-8,20*(4^(k+1)));
        meshes[k + 1].reserve(20 * std::pow(4, resolution) - 8, 20 * std::pow(4, resolution) - 8);
      } catch (const std::exception& ex) {
        std::cout << "Linus is haunting you! (" << k << ")" << std::endl;
      }
      subdivide_mesh(meshes[k], meshes[k + 1]);
    }

    Mesh<T>::triangles = meshes.back().triangles;
    Mesh<T>::vertices = meshes.back().vertices;

    for (auto &it: Mesh<T>::vertices) {
      it *= radius;
      it += origin;
    }

  };
};

}
#endif // EM_ORIGINSPHERE_H
