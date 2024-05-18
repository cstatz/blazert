#include <iostream>
#include <fstream>
#include <blazert/blazert.h>
#include "cnpy.h"
#include <cmath>

#include "OriginSphereSector.h"
#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;
using namespace blazert;
using namespace emrt;
using namespace emrt::origin;

template<typename T> class OriginConfig {
public:
  blazert::Vec3r<T> origin, direction;
  T radius, cut_angle;
  std::string points_fname, tris_fname;
  uint32_t level;

  explicit OriginConfig(char * fname) {

    json config_data = json::parse(std::ifstream(fname));
    points_fname = config_data["points_file"];
    tris_fname = config_data["tris_file"];
    radius = config_data["radius"];
    cut_angle = config_data["cut_angle"];
    level = config_data["level"];

    for (size_t i=0; i<config_data["origin"].size(); i++) {
      origin[i] = config_data["origin"][i];
    }
    for (size_t i=0; i<config_data["direction"].size(); i++) {
      direction[i] = config_data["direction"][i];
    }
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const OriginConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "points_file": )" << config.points_fname << ",\n";
  stream << R"(  "tris_file": )" << config.tris_fname << ",\n";
  stream << R"(  "origin": [ )" << config.origin[0] << ", " << config.origin[1] << ", " << config.origin[2] << "],\n";
  stream << R"(  "direction": [ )" << config.direction[0] << ", " << config.direction[1] << ", " << config.direction[2] << "],\n";
  stream << R"(  "radius": )" << config.radius << ",\n";
  stream << R"(  "level": )" << config.level << ",\n";
  stream << R"(  "cut_angle": )" << config.cut_angle << "\n";

  stream << "}";
  return stream;
}

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Simulation file missing (json)" << std::endl;
    return 1;
  }

  auto config = OriginConfig<double>(argv[1]);

  std::cout << config << std::endl;

  SphereSector<double> ss{config.level, config.origin, config.radius, config.direction, config.cut_angle};

  cnpy::npy_save(config.points_fname, (double *) &(ss.vertices[0][0]),{ss.vertices.size(), 3},"w");
  cnpy::npy_save(config.tris_fname, (uint32_t *) &(ss.triangles[0]),{ss.triangle_count(), 3},"w");
  return 0;
}

