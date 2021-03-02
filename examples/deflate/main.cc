#include <iostream>
#include <vector>
#include <fstream>
#include <blazert/blazert.h>
#include "cnpy.h"
#include <cmath>
#include "constants.h"
#include "OriginSphere.h"

#include <nlohmann/json.hpp>


// for convenience
using json = nlohmann::json;
using namespace blazert;

template<typename T> class DeflateConfig {
public:
  std::string points_fname, tris_fname, spheres_centers_fname, spheres_radii_fname;
  T radius;
  uint32_t level;
  Vec3r<T> origin;

  explicit DeflateConfig(char * fname) {

    json config_data = json::parse(std::ifstream(fname));

    points_fname = config_data["points_file"];
    tris_fname = config_data["tris_file"];
    radius = config_data["radius"];
    level = config_data["level"];

    for (size_t i=0; i<config_data["origin"].size(); i++) {
      origin[i] = config_data["origin"][i];
    }

    spheres_centers_fname = config_data["spheres_centers_file"];
    spheres_radii_fname = config_data["spheres_radii_file"];
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const DeflateConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "points_file": )" << config.points_fname << ",\n";
  stream << R"(  "tris_file": )" << config.tris_fname << ",\n";
  stream << R"(  "origin": [ )" << config.origin[0] << ", " << config.origin[1] << ", " << config.origin[2] << "],\n";
  stream << R"(  "spheres_centers_file": )" << config.spheres_centers_fname << ",\n";
  stream << R"(  "spheres_radii_file": )" << config.spheres_radii_fname << ",\n";
  stream << R"(  "radius": )" << config.radius << ",\n";
  stream << R"(  "level": )" << config.level << "\n";
  stream << "}";
  return stream;
}

template<typename T, typename P> std::unique_ptr<T> read_1d_array(const std::string &fname) {

  auto arr = cnpy::npy_load(fname);
  auto data = std::make_unique<T>(arr.num_vals);

  for (size_t i=0; i<arr.num_vals; i++) {
    (*data)[i] = arr.data<P>()[i];
  }

  return data;
}

template<typename T, typename P> std::unique_ptr<T> read_2d_array(std::string &fname) {

  auto arr = cnpy::npy_load(fname);
  auto data = std::make_unique<T>(arr.shape[0]);
  //data->resize(arr.shape[0];

  std::cout << arr.shape[0] << ", " << arr.shape[1] << std::endl;
  for (size_t i=0; i<arr.shape[0]; i++) {
    for (size_t k=0; k<arr.shape[1]; k++) {
        (*data)[i][k] = arr.data<P>()[k + i*arr.shape[1]];
    }
  }

  return data;
}

template<typename T> T signed_volume_of_triangle_x_6(const Vec3r<T> &p1, const Vec3r<T> &p2, const Vec3r<T> &p3, const Vec3r<T> o) {
  return dot(p1-o, cross(p2-o, p3-o));
}

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Simulation file missing (json)" << std::endl;
    return 1;
  }

  auto config = DeflateConfig<double>(argv[1]);

  std::cout << config << std::endl;

  blazert::Scene<double> scene;

  std::unique_ptr<Vec3rList<double>> vertices;
  std::unique_ptr<Vec3iList> triangles;

  emrt::origin::Sphere<double> ss{config.level, config.origin, config.radius};

  std::unique_ptr<Vec3rList<double>> spheres_centers;
  std::unique_ptr<std::vector<double>> spheres_radii;

  if (config.spheres_centers_fname.length() > 0) {
    spheres_centers = read_2d_array<blazert::Vec3rList<double>, double>(config.spheres_centers_fname);
    spheres_radii = read_1d_array<std::vector<double>, double>(config.spheres_radii_fname);
    scene.add_spheres(*spheres_centers,*spheres_radii);
  };

  scene.commit();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (auto &it: ss.vertices) {
    const Ray<double> r{it, config.origin - it};
    RayHit<double> rh;

    if (intersect1(scene, r, rh)) {
      it += r.direction*rh.hit_distance;
    }
  }

  // Compute boundary volume
  double boundary_volume{0.};
  for (uint32_t i = 0; i < ss.triangle_count(); ++i) {
    const auto &t = ss.triangles;
    boundary_volume += std::abs(signed_volume_of_triangle_x_6(ss.vertices[t[i*3 + 0]],
                                                              ss.vertices[t[i*3 + 1]],
                                                              ss.vertices[t[i*3 + 2]], config.origin));
  }
  boundary_volume *= emrt::constants::one_over_six<double>;
  std::cout << "Boundary Volume: " << boundary_volume << std::endl;

  // Compute spheres volume
  double spheres_volume{0.};
  for (auto &it: *spheres_radii) {
    spheres_volume += it*it*it;
  }
  spheres_volume *= (4./3. * emrt::constants::pi<double>);

  std::cout << "Spheres Volume: " << spheres_volume << std::endl;
  std::cout << "Porosity: " << double((boundary_volume - spheres_volume)/boundary_volume) << std::endl;

  cnpy::npy_save(config.points_fname, (double *) &(ss.vertices[0][0]),{ss.vertices.size(), 3},"w");
  cnpy::npy_save(config.tris_fname, (uint32_t *) &(ss.triangles[0]),{ss.triangle_count(), 3},"w");

  return 0;
}