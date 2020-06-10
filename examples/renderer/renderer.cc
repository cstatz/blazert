#include <cstdio>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <functional>

#include <blazert/blazert.h>
#include "../common/tiny_obj_loader.h"

using namespace blazert;
using ft      = float;

template<typename T>
struct Mesh {
  Vec3rList<T> vertices;
  Vec3iList faces;/// [xyz] * num_vertices
  std::vector<unsigned int> material_ids;
  Mat3rList<T> facevarying_uvs;    /// [xyz] * 3(triangle) * num_faces -> this may be Mat2rList<T>
};

template<typename T>
bool LoadObj(Mesh<T> &mesh, std::vector<tinyobj::material_t> &materials, const char *filename, float scale, const char *mtl_path) {

  std::vector<tinyobj::shape_t> shapes;
  std::string err = tinyobj::LoadObj(shapes, materials, filename, mtl_path);

  if (!err.empty()) {
    std::cerr << err << std::endl;
    return false;
  }

  std::cout << "[LoadOBJ] # of shapes in .obj : " << shapes.size() << std::endl;
  std::cout << "[LoadOBJ] # of materials in .obj : " << materials.size()
            << std::endl;

  size_t num_vertices = 0;
  size_t num_faces = 0;
  for (size_t i = 0; i < shapes.size(); i++) {
    std::cout << "  shape[" << i << "].name = " << shapes[i].name.c_str() << "\n";
    std::cout << "  shape[" << i << "].indices: " << shapes[i].mesh.indices.size() << "\n";
    assert((shapes[i].mesh.indices.size() % 3) == 0);
    std::cout << "  shape[" << i << "].vertices: " << shapes[i].mesh.positions.size() << "\n";
    assert((shapes[i].mesh.positions.size() % 3) == 0);
    std::cout << "  shape[" << i << "].normals: " << shapes[i].mesh.normals.size() << "\n";
    assert((shapes[i].mesh.normals.size() % 3) == 0);

    num_vertices += shapes[i].mesh.positions.size() / 3;
    num_faces += shapes[i].mesh.indices.size() / 3;
  }
  std::cout << "[LoadOBJ] # of faces: " << num_faces << std::endl;
  std::cout << "[LoadOBJ] # of vertices: " << num_vertices << std::endl;

  size_t vertexIdxOffset = 0;

  for (size_t i = 0; i < shapes.size(); i++) {
    // Faces + Mats
    for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {
      mesh.faces.push_back(Vec3ui{shapes[i].mesh.indices[3 * f + 0], shapes[i].mesh.indices[3 * f + 1], shapes[i].mesh.indices[3 * f + 2]} + vertexIdxOffset);
      mesh.material_ids.push_back(shapes[i].mesh.material_ids[f]);
    }

    // Vertices
    for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
      mesh.vertices.push_back(scale * Vec3r<ft>{shapes[i].mesh.positions[3 * v + 0], shapes[i].mesh.positions[3 * v + 1], shapes[i].mesh.positions[3 * v + 2]});
    }

    vertexIdxOffset = mesh.vertices.size();

    if (shapes[i].mesh.texcoords.size() > 0) {
      for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {

        const int f0 = shapes[i].mesh.indices[3 * f + 0];
        const int f1 = shapes[i].mesh.indices[3 * f + 1];
        const int f2 = shapes[i].mesh.indices[3 * f + 2];

        const Vec3r<ft> n0{shapes[i].mesh.texcoords[2 * f0 + 0], shapes[i].mesh.texcoords[2 * f0 + 1], 0.};
        const Vec3r<ft> n1{shapes[i].mesh.texcoords[2 * f1 + 0], shapes[i].mesh.texcoords[2 * f1 + 1], 0.};
        const Vec3r<ft> n2{shapes[i].mesh.texcoords[2 * f2 + 0], shapes[i].mesh.texcoords[2 * f2 + 1], 0.};

        Mat3r<ft> n{};
        blaze::column<0UL>(n) = n0;
        blaze::column<1UL>(n) = n1;
        blaze::column<2UL>(n) = n2;
        mesh.facevarying_uvs.emplace_back(n);
      }
    }
  }

  return true;
}

struct Camera {
    Vec3r<ft> eye;
    Vec3r<ft> dir;
    Vec3r<ft> up;
    ft  fov;
    explicit Camera(const Vec3r<ft> &eye, const Vec3r<ft> &dir, const Vec3r<ft>& up, const ft fov) noexcept : eye(eye), dir(dir), up(up), fov(fov) {};
};

template <typename BVH>
void render(const Camera& camera, const BVH &bvh, ft* pixels, size_t width, size_t height)
{
    const auto dir = blaze::normalize(camera.dir);
    const auto image_u = blaze::normalize(blaze::cross(dir, camera.up));
    const auto image_v = blaze::normalize(blaze::cross(image_u, dir));
    const auto image_w = std::tan(camera.fov * ft(3.14159265 * (1.0 / 180.0) * 0.5));
    const auto ratio = ft(height) / ft(width);
    const auto image_u_ = image_u * image_w;
    const auto image_v_ = image_v * image_w * ratio;
    
    for(size_t i = 0; i < width; ++i) {
        for(size_t j = 0; j < height; ++j) {
            size_t index = 3 * (width * j + i);

            auto u = 2 * (i + ft(0.5)) / ft(width)  - ft(1);
            auto v = 2 * (j + ft(0.5)) / ft(height) - ft(1);

            const Ray<ft> ray(camera.eye, blaze::normalize(image_u_ * u + image_v_ * v + dir));
            RayHit<ft> rayhit;
            if (!traverse(bvh, ray, rayhit)) {
              pixels[index] = pixels[index + 1] = pixels[index + 2] = 0;
            } else {
              pixels[index    ] = std::abs(rayhit.normal[0]);
              pixels[index + 1] = std::abs(rayhit.normal[1]);
              pixels[index + 2] = std::abs(rayhit.normal[2]);
            }
        }
    }
}

int main(int argc, char** argv) {

  std::string objFilename = "models/cornellbox_suzanne_lucy.obj";
  std::string mtlPath = "models/";
    const char* output_file  = "render.ppm";

  auto *mesh = new Mesh<ft>;

  std::vector<tinyobj::material_t> materials;
  const bool ret = LoadObj(*mesh, materials, objFilename.c_str(), ft(1.0), mtlPath.c_str());

    Camera camera{{0, 5, 10}, {0., 0., -1.}, {0, 1, 0}, 60};

    size_t width  = 1080;
    size_t height = 720;

    TriangleMesh triangles(mesh->vertices, mesh->faces);
    BVH bvh(triangles);
    SAHBinnedBuilder builder;
    builder.build(bvh);

    auto pixels = std::make_unique<ft[]>(3 * width * height);
    render(camera, bvh, pixels.get(), width, height);

    std::ofstream out(output_file, std::ofstream::binary);
    out << "P6 " << width << " " << height << " " << 255 << "\n";
    for(size_t j = height; j > 0; --j) {
        for(size_t i = 0; i < width; ++i) {
            size_t index = 3* (width * (j - 1) + i);
            uint8_t pixel[3] = {
                static_cast<uint8_t>(std::max(std::min(pixels[index    ] * 255, ft(255)), ft(0))),
                static_cast<uint8_t>(std::max(std::min(pixels[index + 1] * 255, ft(255)), ft(0))),
                static_cast<uint8_t>(std::max(std::min(pixels[index + 2] * 255, ft(255)), ft(0)))
            };
            out.write(reinterpret_cast<char*>(pixel), sizeof(uint8_t) * 3);
        }
    }
    return 0;
}
