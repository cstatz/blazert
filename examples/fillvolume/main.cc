#include <iostream>
#include <vector>
#include <fstream>
#include <blazert/blazert.h>
#include "cnpy.h"
#include <complex>
#include <cmath>

#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;
using namespace blazert;

void progress_bar(unsigned long tick, unsigned long total, unsigned long width = 100) {
  double ratio = 100.0 * double(tick) / double(total);
  unsigned count = std::floor(width * tick / total);
  std::string bar(width, ' ');
  std::fill(bar.begin(), bar.begin() + count, '+');
  std::cout << "[ " << ratio << "%% ] [ " << bar << " ]" << (tick == total ? '\n' : '\r');
  std::fflush(stdout);
}

template<typename T> class FVConfig {
public:
  std::string points_fname, tris_fname, spheres_centers_fname, spheres_radii_fname, ids_fname, output_fname;
  std::vector<T> x, y, z;
  Vec3r<T> tp;

  explicit FVConfig(char * fname) {

    json config_data = json::parse(std::ifstream(fname));

    points_fname = config_data["points_file"];
    tris_fname = config_data["tris_file"];
    spheres_centers_fname = config_data["spheres_centers_file"];
    spheres_radii_fname = config_data["spheres_radii_file"];
    ids_fname = config_data["ids_file"];
    output_fname = config_data["results_file"];

    for (const auto &it: config_data["x_axis"]) x.emplace_back(it);
    for (const auto &it: config_data["y_axis"]) y.emplace_back(it);
    for (const auto &it: config_data["z_axis"]) z.emplace_back(it);

    for (size_t i=0; i<config_data["test_point"].size(); i++) {
      tp[i] = config_data["test_point"][i];
    }
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const FVConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "results_file": )" << config.output_fname << ",\n";
  stream << R"(  "points_file": )" << config.points_fname << ",\n";
  stream << R"(  "tris_file": )" << config.tris_fname << ",\n";
  stream << R"(  "spheres_centers_file": )" << config.spheres_centers_fname << ",\n";
  stream << R"(  "spheres_radii_file": )" << config.spheres_radii_fname << ",\n";
  stream << R"(  "ids_file": )" << config.ids_fname << "\n";
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

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Simulation file missing (json)" << std::endl;
    return 1;
  }

  auto config = FVConfig<double>(argv[1]);

  std::cout << config << std::endl;

  std::vector<size_t> id_offsets;
  id_offsets.emplace_back(0);

  blazert::Scene<double> scene;

  std::unique_ptr<Vec3rList<double>> vertices;
  std::unique_ptr<Vec3iList> triangles;

  if (config.points_fname.length() > 0) {
    vertices = read_2d_array<blazert::Vec3rList<double>, double>(config.points_fname);
    triangles = read_2d_array<blazert::Vec3iList, uint32_t>(config.tris_fname);
    id_offsets.emplace_back((*triangles).size() + id_offsets.back());
    scene.add_mesh(*vertices, *triangles);
  }

  std::unique_ptr<Vec3rList<double>> spheres_centers;
  std::unique_ptr<std::vector<double>> spheres_radii;

  if (config.spheres_centers_fname.length() > 0) {
    spheres_centers = read_2d_array<blazert::Vec3rList<double>, double>(config.spheres_centers_fname);
    spheres_radii = read_1d_array<std::vector<double>, double>(config.spheres_radii_fname);
    id_offsets.emplace_back((*spheres_centers).size() + id_offsets.back());
    scene.add_spheres(*spheres_centers,*spheres_radii);
  };

  scene.commit();

  auto ids = read_1d_array<std::vector<int32_t>, int32_t>(config.ids_fname);

  std::vector<int32_t> id_array;
  id_array.resize(config.x.size()*config.y.size()*config.z.size());

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (size_t k=0; k<config.z.size(); k++) {
    for (size_t j=0; j<config.y.size(); j++) {
      for (size_t i=0; i<config.x.size(); i++) {
        const Vec3r<double> p{config.x[i], config.y[j], config.z[k]};
        const blazert::Ray<double> ray{p, config.tp-p, 0.00001};
        blazert::RayHit<double> rayhit;
        if (intersect1(scene, ray, rayhit)) {
          auto fid = rayhit.prim_id + id_offsets[rayhit.geom_id];
          if (dot(rayhit.normal, ray.direction) > 0.) id_array[i + j*config.x.size() + k*config.x.size()*config.y.size()] = (*ids)[fid];
        }
      }
    }
    progress_bar(k+1, config.z.size());
  }

  cnpy::npy_save(config.output_fname, (int32_t *) &(id_array[0]),{config.x.size(), config.y.size(), config.z.size()},"w");
  return 0;
}

