#include "blazert/blazert.h"
#include "cnpy.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

#include "emrtpo_data.h"
#include "emrtpo_data_header.h"

using namespace blazert;
using namespace emrt;

using Vec2st = blaze::StaticVector<size_t, 2UL, blaze::columnVector, A_, P_>;


class WaveFrontPoint {
public:
  Vec3r<double> point{};
  size_t kind{};
  size_t origin_id{}, endpoint_id{};
  double origin_u{}, origin_v{};
};

std::optional<WaveFrontPoint> wavefront_from_raypath(const VectorEMRTPOData<double> &r, double d) {

  if (r.ray_tracing_propagation_length < d) {
    return std::nullopt;
  }
  if (r.endpoints.size() < 1) {
    return std::nullopt;
  }

  Vec3r<double> A{r.origin};
  double l{0};

  for (size_t i = 0; i < r.endpoints.size(); i++) {
    auto it = r.endpoints[i];
    double ls{length(it - A) * r.iors[i]};
    if ((d >= l) and (d <= (l + ls))) {
      // We are in the segment
      // Interpolate using the adjacent points and the adjacent distances
      // (B - A) * (d - a) / (b - a) + A
      WaveFrontPoint w;
      w.point = (it - A) * (d - l) / ls + A;
      w.origin_id = r.origin_fid;
      w.endpoint_id = r.endpoint_fid;
      w.origin_u = r.origin_u;
      w.origin_v = r.origin_v;
      w.kind = r.kinds[i];
      return w;
    }
    l += ls;
    A = it;
  }

  // If for some reason (this should have been caught by now) point has not been found: return nullopt;
  return std::nullopt;
};

int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Simulation file missing (json)" << std::endl;
    return 1;
  }

  std::ifstream is(argv[1], std::ifstream::binary);
  double d = atof(argv[2]);

  if (is) {
    // get length of file:
    is.seekg(0, std::ifstream::end);

    size_t length = static_cast<size_t>(is.tellg());
    size_t n_good_rays = 0;

    is.seekg(static_cast<long long int>(length - sizeof(n_good_rays)));
    is.read((char *) &n_good_rays, sizeof(n_good_rays));
    is.seekg(0, std::ifstream::beg);

    std::cout << "Number of rays: " << n_good_rays << std::endl;

    VectorEMRTPOHeader h;
    h.read(is);

    size_t n_wf_points = 0;
    std::vector<Vec3r<double>> points;
    std::vector<Vec2r<double>> origin_uvs;
    std::vector<Vec2st> ids;
    std::vector<size_t> kinds;

    points.reserve(n_good_rays * 10);
    origin_uvs.reserve(n_good_rays * 10);
    ids.reserve(n_good_rays * 10);
    kinds.reserve(n_good_rays * 10);

    for (size_t i = 0; i < n_good_rays; i++) {
      VectorEMRTPOData<double> r;
      r.read(is);
      if (auto o = wavefront_from_raypath(r, d)) {
        n_wf_points++;
        points.emplace_back(o->point);
        origin_uvs.emplace_back(Vec2r<double>{o->origin_u, o->origin_v});
        ids.emplace_back(Vec2st{o->origin_id, o->endpoint_id});
        kinds.emplace_back(o->kind);
      }
    }

    std::string prefix = "arr_";
    cnpy::npy_save(prefix + "points_" + std::to_string(d) + ".npy", (double *) &(points[0][0]), {n_wf_points, 3}, "w");
    cnpy::npy_save(prefix + "uvs_" + std::to_string(d) + ".npy", (double *) &(origin_uvs[0][0]), {n_wf_points, 2}, "w");
    cnpy::npy_save(prefix + "ids_" + std::to_string(d) + ".npy", (size_t *) &(ids[0][0]), {n_wf_points, 2}, "w");
    cnpy::npy_save(prefix + "kinds_" + std::to_string(d) + ".npy", (size_t *) &(kinds[0]), {n_wf_points}, "w");

    std::cout << "Done." << std::endl;
    is.close();
  }
  return 0;
}
