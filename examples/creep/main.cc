#include <iostream>
#include <vector>
#include <fstream>
#include <blazert/blazert.h>
#include "cnpy.h"
#include <complex>
#include <cmath>

#include <nlohmann/json.hpp>
#include "raypath.h"
#include "raypath_header.h"

// for convenience
using json = nlohmann::json;
using namespace blazert;
using namespace emrt;
using Vec4ui = blaze::StaticVector<unsigned int, 4UL, blaze::columnVector, A_, P_>;
using Vec4iList = std::vector<Vec4ui, blaze::AlignedAllocator<Vec4ui>>;

constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double rad2deg = 180./pi;
constexpr double deg2rad = pi/180.;

template<typename T>
inline T uniform(T min, T max) {
  return min + T(std::rand()) / T(RAND_MAX) * (max - min);
}

void progress_bar(unsigned long tick, unsigned long total, unsigned long width = 100) {
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
  T resolution, min_distance;
  std::string points_fname, tris_fname, output_fname;
  bool binary_output, output_all_rays;
  unsigned int seed;

  explicit CreepConfig(char * fname) {

    json config_data = json::parse(std::ifstream(fname));
    points_fname = config_data["points_file"];
    tris_fname = config_data["tris_file"];
    output_fname = config_data["results_file"];
    resolution = config_data["resolution"];
    min_distance = config_data["min_distance"];
    binary_output = config_data["binary_output"];

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
  stream << R"(  "resolution": )" << config.resolution << ",\n";
  stream << R"(  "min_distance": )" << config.min_distance << ",\n";
  stream << R"(  "binary_output": )" << config.binary_output << "\n";

  stream << "}";
  return stream;
}

template<typename T, typename P> T read_1d_array(const std::string &fname) {

  auto arr = cnpy::npy_load(fname);
  T data(arr.num_vals);

  for (size_t i=0; i<arr.num_vals; i++) {
    data[i] = arr.data<P>()[i];
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

template<typename T> blaze::StaticMatrix<T, 3UL, 3UL> axisangle2rotmat(const Vec3r<T> &axis, T angle) {

  /**
  = c*
  1	0	0
  0	1	0
  0	0	1
  + t*
  x*x	x*y	x*z
  x*y	y*y	y*z
  x*z	y*z	z*z
  + s*
   0	-z	 y
   z	 0	-x
  -y	 x	 0
  */

  const T c = std::cos(angle/180.*pi);
  const T s = std::sin(angle/180.*pi);
  const T t = T(1.) - c;

  blaze::IdentityMatrix<T> I( 3UL );
  blaze::StaticMatrix<T, 3UL, 3UL> A = blaze::outer(axis, axis);
  blaze::StaticMatrix<T, 3UL, 3UL> B{{T(0.), -axis[2], axis[1]}, {axis[2], T(0.), -axis[0]}, {-axis[1], axis[0], T(0.)}};
  auto m = c * I + t * A + s * B;
  std::cout << axis << m << std::endl;
  return m;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Simulation file missing (json)" << std::endl;
    return 1;
  }

  auto config = CreepConfig<double>(argv[1]);

  std::cout << config << std::endl;

  auto vertices = read_2d_array<blazert::Vec3rList<double>, double>(config.points_fname);
  auto triangles = read_2d_array<blazert::Vec3iList, uint32_t>(config.tris_fname);

  blazert::Scene<double> scene;
  [[maybe_unused]] unsigned int prim_id = scene.add_mesh(*vertices, *triangles);
  scene.commit();

  std::vector<RayPath<double>> ray_paths;

  std::cout << "Starting simulation:" << std::endl;

  // TODO: Trace from O to T
  const blazert::Ray<double> ray(config.origin, config.target - config.origin, config.min_distance);
  blazert::RayHit<double> rayhit;

  if (intersect1(scene, ray, rayhit)) {
    // Trace along the surface
    //std::cout << "I1" << std::endl;
    // TODO: Half distance -> Ot
    const auto ot = ray.origin + ray.direction * rayhit.hit_distance * 0.5;
    std::cout << "--> Ot: " << ot << std::endl;
    // TODO: Define Up

    Vec3r<double> up{0., 0., 1.};
    const auto dir = normalize(config.origin - config.target);
    {
      const auto tmp_axis = normalize(cross(dir, up));
      up = normalize(cross(tmp_axis, dir));
    }
    std::cout << "--> Up: "<< up << std::endl;

    // TODO: Cleanup the double in for-loop-mess
    for (size_t u=0; u<config.resolution; u++) {

      RayPath<double> rp(u, 0,0., 0., config.origin, 0., true, true);
      Vec3r<double> pp = rp.origin;

      const double up_angle = u * 360./config.resolution;
      std::cout << "UA: " << up_angle << std::endl;
      const auto test_up = axisangle2rotmat<double>(dir, up_angle) * up;
      std::cout << "--> TU: "<< test_up << std::endl;
      const auto axis = normalize(cross(test_up, dir));
      //std::cout << "--> AX: "<< axis << std::endl;

      std::cout << "--> AX: "<< axis << std::endl;

      for (size_t d=0; d<config.resolution; d++) {
        const double dir_angle = d * 180./config.resolution;
        std::cout << "DA: " << dir_angle << std::endl;
        std::cout << "--> AX2: "<< axis << std::endl;
        const auto test_dir = axisangle2rotmat<double>(axis, dir_angle) * dir;
        std::cout << "--> D: "<< test_dir << std::endl;
        const blazert::Ray<double> test_ray(ot, test_dir, config.min_distance);//, config.min_distance);
        blazert::RayHit<double> test_rayhit;

        if (intersect1(scene, test_ray, test_rayhit)) {
          //std::cout << "I2" << std::endl;
          // TODO: Add the endpoint of the intersection to the raypath

          const auto ep = test_ray.origin + test_rayhit.hit_distance * test_ray.direction;
          rp.add_segment(ep, length(ep-pp), 2., 0.);
          pp = ep;

          const blazert::Ray<double> los_ray(ep, config.target - ep, config.min_distance);
          blazert::RayHit<double> los_rayhit;

          if (!intersect1(scene, los_ray, los_rayhit)) {
            //std::cout << "I3" << std::endl;
            rp.add_segment(config.target, length(config.target-ep), 1., 0.);
            break;
          }
        }
        else {
          std::cout << "Something wrong ..." << std::endl;
          break;
        }
        // TODO: Sum up segment lengths. For each segment test visibility to T. If visible, add distance and finalize
        // TODO: Handle cases where we hit the surface from the outside at a distance (probably no more surface guided propagation ...)
      }
      rp.finalize(1,0.,0.);
      ray_paths.emplace_back(rp);

      // TODO: Start over with new up
    }
    // TODO: find minimum of accumulated distance

    // TODO: Parallelize over ups
  }
  else {
    // LoS
    std::cout << "LOS" << std::endl;
    RayPath<double> rp(-1, 0, 0., 0., config.origin, 0., true, true);
    rp.origin = config.origin;
    rp.add_segment(config.target, length(config.target-config.origin), 1., 0.);
    rp.finalize(1,0.,0.);
    ray_paths.emplace_back(rp);
  }

  std::ofstream outfile;

  if (config.binary_output) {
    outfile.open(config.output_fname, std::ios::trunc | std::ios::binary);

    RayPathHeader rh;
    rh.write(outfile);

    size_t n_good_rays = 0;

    for (auto &it : ray_paths) {
        it.write(outfile);
        n_good_rays++;
    }

    outfile.write((char *) (&n_good_rays), sizeof(n_good_rays));
  }
  else {
    outfile.open (config.output_fname, std::ios::trunc);
    outfile << "[" << std::endl;
    for (auto &it: ray_paths) {
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

