#pragma once
#ifndef EM_ORIGINSPHERESECTOR_H
#define EM_ORIGINSPHERESECTOR_H

#include <blazert/blazert.h>
#include "OriginSphere.h"
#include "OriginMesh.h"


namespace emrt::origin {


template<typename T>
class SphereSector : public Mesh<T>
{
private:
  const uint32_t n_ray;

public:
  SphereSector() = delete;
  /***
   * Constructor for SphereSector
   *
   * @param resolution defines the amount of rays which are created
   * @param dir Eigen::Vector3D defining the direction in which a finer mesh
   * should be created
   * @param angle_cut angle which defines how much around the direction vector is
   * a fine mesh (Pi defines a half sphere)
   */
  explicit SphereSector(uint32_t resolution, const blazert::Vec3r<T> &origin, const double radius, const blazert::Vec3r<T> &dir, const double angle_cut)
    : n_ray(T(20) * std::pow(T(4), resolution))
  {
    // std::cout << "dir vector = " << *dir << std::endl;
    // std::cout << "N = " << N << std::endl;
    // std::cout << "20*(4^(resolution))-8 = " << 20*std::pow(4,resolution)-8 <<
    // std::endl;
    std::vector<Mesh<T>> meshes;

    // for (auto &m : meshes) m.clear();

    meshes.emplace_back(Mesh<T>());
    initialize_icosahedron(meshes.back());

    for (uint32_t i = 0; i < resolution; i++) {
      meshes.emplace_back(Mesh<T>());
    }
    size_t tmp = 0;
    for (uint32_t k = 0; k < resolution; ++k) {
      try {
        meshes[k + 1].reserve(20 * std::pow(4, resolution) - 8, 20 * std::pow(4, resolution) - 8);
      } catch (const std::exception& ex) {
        std::cout << "Linus is haunting you! (" << k << ")" << std::endl;
      }
      if (tmp <= 3) {
        tmp++;
        subdivide_mesh(meshes[k], meshes[k + 1]);
      } else
        subdivide_mesh(meshes[k], meshes[k + 1], dir, angle_cut);
    }

    Mesh<T>::triangles = meshes.back().triangles;
    Mesh<T>::vertices = meshes.back().vertices;

    for (auto &it: Mesh<T>::vertices) {
      it *= radius;
      it += origin;
    }
  };

  /***
   * returns the amount of rays which would be required if the whole sphere would
   * be meshed. This is necessary because we still need to know how much energy is
   * carried by one ray with respect to the entire sphere.
   *
   * @return unsigned integer
   */
  uint32_t vertices_count() const {return n_ray;};
};

}
#endif // EM_ORIGINSPHERESECTOR_H
