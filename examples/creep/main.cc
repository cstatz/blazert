#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

#include <blazert/blazert.h>
#include <nlohmann/json.hpp>

#include "cnpy.h"
#include "raypath.h"
#include "raypath_header.h"

using json = nlohmann::json;
using namespace blazert;
using namespace emrt;

template<typename T> constexpr T pi = T(3.141592653589793238462643383279502884);
template<typename T> constexpr T rad2deg = T(180.)/pi<T>;
template<typename T> constexpr T deg2rad = pi<T>/T(180.);

void progress_bar(const unsigned long tick, const unsigned long total, const unsigned long width = 100) {
  double ratio = 100.0 * double(tick) / double(total);
  unsigned count = std::floor(width * tick / total);
  std::string bar(width, ' ');
  std::fill(bar.begin(), bar.begin() + count, '+');
  std::cout << "[ " << ratio << "%% ] [ " << bar << " ]" << (tick == total ? '\n' : '\r');
  std::fflush(stdout);
}

template<typename T> class CreepConfig {
public:
  blazert::Vec3r<T> origin, target;
  size_t up_resolution, path_resolution;
  T min_distance, ior;
  std::string points_fname, tris_fname, output_fname;
  bool binary_output;

  CreepConfig() =delete;
  explicit CreepConfig(char * fname) {

    json config_data = json::parse(std::ifstream(fname));
    points_fname = config_data["points_file"];
    tris_fname = config_data["tris_file"];
    output_fname = config_data["results_file"];
    up_resolution = config_data["up_resolution"];
    path_resolution = config_data["path_resolution"];
    min_distance = config_data["min_distance"];
    binary_output = config_data["binary_output"];
    ior = config_data["ior"];

    for (size_t i=0; i<config_data["origin"].size(); i++) {
      origin[i] = config_data["origin"][i];
    }
    for (size_t i=0; i<config_data["target"].size(); i++) {
      target[i] = config_data["target"][i];
    }
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const CreepConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "results_file": )" << config.output_fname << ",\n";
  stream << R"(  "points_file": )" << config.points_fname << ",\n";
  stream << R"(  "tris_file": )" << config.tris_fname << ",\n";
  stream << R"(  "origin": [ )" << config.origin[0] << ", " << config.origin[1] << ", " << config.origin[2] << "],\n";
  stream << R"(  "target": [ )" << config.target[0] << ", " << config.target[1] << ", " << config.target[2] << "],\n";
  stream << R"(  "up_resolution": )" << config.up_resolution << ",\n";
  stream << R"(  "path_resolution": )" << config.path_resolution << ",\n";
  stream << R"(  "min_distance": )" << config.min_distance << ",\n";
  stream << R"(  "ior": )" << config.ior << ",\n";
  stream << R"(  "binary_output": )" << config.binary_output << "\n";

  stream << "}";
  return stream;
}

template<typename T, typename P> std::unique_ptr<T> read_2d_array(const std::string &fname) {

  const auto arr = cnpy::npy_load(fname);
  auto data = std::make_unique<T>(arr.shape[0]);

  for (size_t i=0; i<arr.shape[0]; i++) {
    for (size_t k=0; k<arr.shape[1]; k++) {
        (*data)[i][k] = arr.data<P>()[k + i*arr.shape[1]];
    }
  }

  return data;
}

template<typename T> blaze::StaticMatrix<T, 3UL, 3UL> axisangle2rotmat(const Vec3r<T> &axis, const T angle) {

  /**  The rotation matrix is constructed as follows:
   *
   *    c = cos(angle)
   *    s = sin(angle)
   *    t = 1. - c
   *
   *    m = c * I + t * A + s * B
   *
   *            1  0  0         x^2 x*y x*z         0 -z  y
   *    m = c * 0  1  0  +  t * x*y y^2 y*z  +  s * z  0 -x
   *            0  0  1         x*z y*z z^2        -y  x  0
   *
  */

  const T a = angle * deg2rad<T>;
  const T c = std::cos(a);
  const T s = std::sin(a);
  const T t = T(1.) - c;

  const blaze::IdentityMatrix<T> I( 3UL );
  const blaze::StaticMatrix<T, 3UL, 3UL> A = blaze::outer(axis, axis);
  const blaze::StaticMatrix<T, 3UL, 3UL> B{{   T(0.), -axis[2],  axis[1]},
                                           { axis[2],    T(0.), -axis[0]},
                                           {-axis[1],  axis[0],    T(0.)}};
  const auto m = c * I + t * A + s * B;
  return m;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Simulation file missing (json)" << std::endl;
    return 1;
  }

  const auto config = CreepConfig<double>(argv[1]);

  std::cout << config << std::endl;

  const auto vertices = read_2d_array<blazert::Vec3rList<double>, double>(config.points_fname);
  const auto triangles = read_2d_array<blazert::Vec3iList, uint32_t>(config.tris_fname);

  blazert::Scene<double> scene;
  [[maybe_unused]] const unsigned int prim_id = scene.add_mesh(*vertices, *triangles);
  scene.commit();

  std::vector<RayPath<double>> ray_paths;
  ray_paths.reserve(config.up_resolution);

  std::cout << "Starting simulation:" << std::endl;

  const blazert::Ray<double> ray(config.origin, evaluate(config.target - config.origin), config.min_distance);
  blazert::RayHit<double> rayhit;

  // TODO: Find starting point from tangent to surface

  if (intersect1(scene, ray, rayhit)) {
    // Trace along the surface
    const auto ot = ray.origin + ray.direction * rayhit.hit_distance * 0.5;

    const auto dir = normalize(config.origin - config.target);
    const auto tmp_axis = evaluate(normalize(cross(dir, Vec3r<double>{0., 0., 1.})));
    const auto up = evaluate(normalize(cross(tmp_axis, dir)));

    for (size_t u=0; u<config.up_resolution; u++) {

      RayPath<double> rp(u, 0,0., 0., config.origin, 0., true, true);
      Vec3r<double> pp(rp.origin);

      const double up_angle = u * 360./config.up_resolution;
      const auto test_up = evaluate(axisangle2rotmat<double>(dir, up_angle) * up);
      const auto axis = evaluate(normalize(cross(test_up, dir)));

      for (size_t d=0; d<config.path_resolution; d++) {
        const double dir_angle = d * 180./config.path_resolution;
        const blazert::Ray<double> test_ray(ot, evaluate(axisangle2rotmat<double>(axis, dir_angle) * dir), config.min_distance);
        blazert::RayHit<double> test_rayhit;

        if (intersect1(scene, test_ray, test_rayhit)) {
          const auto ep = test_ray.origin + test_rayhit.hit_distance * test_ray.direction;
          rp.add_segment(ep, length(ep - pp), config.ior, 0.);
          pp = ep;

          const blazert::Ray<double> los_ray(ep, evaluate(config.target - ep), config.min_distance);
          blazert::RayHit<double> los_rayhit;

          if (!intersect1(scene, los_ray, los_rayhit)) {
            // Line-of-Sight
            rp.add_segment(config.target, length(config.target - ep), 1., 0.);
            break;
          }
        }
        else {
          std::cout << "Something wrong ..." << std::endl;
          break;
        }
        // TODO: Handle cases where we hit the surface from the outside at a distance (probably no more surface guided propagation ...)
      }
      rp.finalize(1,0.,0.);
      ray_paths.emplace_back(rp);
      progress_bar(u + 1, config.up_resolution);
    }
    // TODO: find minimum of accumulated distance
    // TODO: Parallelize over ups
  }
  else {
    // Line-of-Sight
    RayPath<double> rp(-1, 0, 0., 0., config.origin, 0., true, true);
    rp.add_segment(config.target, length(config.target - config.origin), 1., 0.);
    rp.finalize(2,0.,0.);
    ray_paths.emplace_back(rp);
  }

  std::ofstream outfile;

  if (config.binary_output) {
    outfile.open(config.output_fname, std::ios::trunc | std::ios::binary);
    if(!outfile) {
      std::cerr << "Error writing outfile.\n";
      return -1;
    }

    RayPathHeader rh;
    rh.write(outfile);

    size_t n_good_rays{0};

    for (const auto &it : ray_paths) {
        it.write(outfile);
        n_good_rays++;
    }

    outfile.write((char *) (&n_good_rays), sizeof(n_good_rays));
  }
  else {
    outfile.open (config.output_fname, std::ios::trunc);
    if(!outfile) {
      std::cerr << "Error writing outfile.\n";
      return -1;
    }

    outfile << "[" << std::endl;
    for (const auto &it: ray_paths) {
        outfile << it;
        if (&it - &*(ray_paths.begin()) < ray_paths.size() - 1) {
          outfile << ", " << std::endl;
        }
    }
    outfile << "]" << std::endl;
  }
  outfile.close();
  std::cout << "Done." << std::endl;
  return 0;
}

