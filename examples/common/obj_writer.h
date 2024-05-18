#pragma once
#ifndef EMRTPO_OBJ_WRITER_H
#define EMRTPO_OBJ_WRITER_H

#include "blazert/datatypes.h"
#include <fstream>

using Vec4ui = blaze::StaticVector<unsigned int, 4UL, blaze::columnVector, A_, P_>;
using Vec4iList = std::vector<Vec4ui, blaze::AlignedAllocator<Vec4ui>>;

template<typename T>
inline T uniform(T min, T max) {
  return min + T(std::rand()) / T(RAND_MAX) * (max - min);
}

template<typename T>
struct Mesh {
  blazert::Vec3rList<T> vertices;
  blazert::Vec3iList triangles;
  Vec4iList ids;
};

template<typename T>
void write_wavefront_obj(const Mesh<T> &mesh, const std::string &filename) {
  std::ofstream file;
  file.open(filename);

  file << "# OBJ file format with ext .obj\n";
  file << "# vertex count = " << mesh.vertices.size() << "\n";
  file << "# face count = " << mesh.vertices.size() << "\n";
  for (const auto &vert : mesh.vertices) {
    file << "v " << vert[0] << " " << vert[1] << " " << vert[2] << "\n";
  }
  for (const auto &face : mesh.triangles) {
    file << "f " << face[0] + 1 << " " << face[1] + 1 << " " << face[2] + 1 << "\n";
  }

  file.close();
}

template<typename T>
void write_wavfront_obj_with_material(const Mesh<T> &mesh, const std::string &filename,
                                      const blazert::Vec3rList<T> &colors) {
  std::ofstream file;
  file.open(filename);

  file << "# OBJ file format with ext .obj\n";
  file << "# vertex count = " << mesh.vertices.size() << "\n";
  file << "# face count = " << mesh.vertices.size() << "\n";
  file << "mtllib " << filename << ".mtl"
       << "\n";

  for (const auto &vert : mesh.vertices) {
    file << "v " << vert[0] << " " << vert[1] << " " << vert[2] << "\n";
  }
  for (uint32_t i = 0; i < mesh.triangles.size(); ++i) {
    file << "usemtl material" << i << '\n';
    file << "f " << mesh.triangles[i][0] + 1 << " " << mesh.triangles[i][1] + 1 << " " << mesh.triangles[i][2] + 1
         << "\n";
  }

  file.close();

  //////// material file:

  std::ofstream mtlfile;
  std::string filenamestr{filename};
  mtlfile.open(filenamestr + ".mtl");

  //T dc = 1./(mesh.triangles.size()-1)
  for (uint32_t i = 0; i < mesh.triangles.size(); ++i) {
    mtlfile << "newmtl material" << i << '\n';
    mtlfile << "   Kd " << colors[i][0] << " " << colors[i][1] << " " << colors[i][2] << "\n\n";
  }

  mtlfile.close();
}

template<typename T>
void write_wavefront_obj_with_material_layers(const Mesh<T> &mesh, const std::string &prefix,
                                              const std::string &filename) {
  std::srand(0);

  std::ofstream file{prefix + filename + ".obj"};

  file << "# OBJ file format with ext .obj\n";
  file << "# vertex count = " << mesh.vertices.size() << "\n";
  file << "# face count = " << mesh.vertices.size() << "\n";
  file << "mtllib " << filename << ".mtl"
       << "\n";

  std::vector<uint32_t> to_layers{};
  for (const auto &id : mesh.ids) {
    to_layers.push_back(id[1]);
  }
  uint32_t n_layer = *std::max_element(to_layers.cbegin(), to_layers.cend());
  std::cout << "n_layer = " << n_layer << "\n";

  // write vertices
  for (const auto &vert : mesh.vertices) {
    file << "v " << vert[0] << " " << vert[1] << " " << vert[2] << "\n";
  }

  // write faces and corresponding materials
  for (uint32_t i = 0; i < mesh.triangles.size(); ++i) {
    file << "usemtl material" << mesh.ids[i][1] << '\n';
    file << "f " << mesh.triangles[i][0] + 1 << " " << mesh.triangles[i][1] + 1 << " " << mesh.triangles[i][2] + 1
         << "\n";
  }
  file.close();

  //////// material file:
  std::ofstream mtlfile{prefix + filename + ".mtl"};

  for (uint32_t i = 0; i <= n_layer; ++i) {
    mtlfile << "newmtl material" << i << '\n';
    mtlfile << "   Kd " << uniform(0., 1.) << " " << uniform(0., 1.) << " " << uniform(0., 1.) << "\n\n";
  }

  mtlfile.close();
}

#endif//EMRTPO_OBJ_WRITER_H
