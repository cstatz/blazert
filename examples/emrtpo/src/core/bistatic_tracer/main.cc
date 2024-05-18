#include <iostream>
#include <vector>
#include <blazert/blazert.h>
#include <complex>
#include <cmath>
#include <nlohmann/json.hpp>

#include "emrtpo_data.h"
#include "emrtpo_data_header.h"
#include "emrtpo_tracer_config.h"

#include "numpy_wrapper.h"
#include "progress_bar.h"

// for convenience
using json = nlohmann::json;
using namespace blazert;
using namespace emrt;
using Vec4ui = blaze::StaticVector<unsigned int, 4UL, blaze::columnVector, A_, P_>;
using Vec4iList = std::vector<Vec4ui, blaze::AlignedAllocator<Vec4ui>>;


template<typename T>
inline T uniform(T min, T max) {
  return min + T(std::rand()) / T(RAND_MAX) * (max - min);
}

template<typename T>
inline Vec3r<T> reflect(const Vec3r<T> &I, const Vec3r<T> &N) {
  return I - 2. * dot(I, N) * N;
}

template<typename T>
inline Vec3r<T> refract(const Vec3r<T> &I, const Vec3r<T> &normal, const T n1, const T n2, const size_t fid) {
  const T n = n1 / n2;
  const T cosI = abs(dot(normal, I));
  const T sinT2 = n * n * (1. - cosI * cosI);
  if (sinT2 > 1.) { std::cout << fid << "," << normal << "," << I << "," << n1 << "," << n2 << " MURKS" << std::endl; }
  const T cosT = std::sqrt(1. - sinT2);
  return n * I + (n * cosI - cosT) * normal;
}

template<typename T>
struct FresnelCoefficients {
    const T gamma_s;
    const T gamma_p;
    const T tau_s;
    const T tau_p;

    FresnelCoefficients(const T r_s, const T r_p, const T t_s, const T t_p) : gamma_s(r_s), gamma_p(r_p), tau_s(t_s), tau_p(t_p) {}
};

template<typename T>
inline FresnelCoefficients<T> fresnel_vector(const Vec3r<T> &H, const Vec3r<T> &norm, const T n1, const T n2) {
  const T tmp = std::abs(dot(normalize(H), normalize(norm)));
  const T cos_theta_i = tmp > 1 ? static_cast<double>(1) : tmp;
  const T theta_i = std::acos(cos_theta_i);
  const T sin_theta_i = std::sin(theta_i);

  const T theta_c = std::asin(n2/n1);
  const bool total_reflection = (theta_i > theta_c) && (n1 > n2);

  if(total_reflection) {
    return FresnelCoefficients<T>(1,1,0,0);
  }

  const T cos_theta_t = std::sqrt(1. - (n1*n1)/(n2*n2) * sin_theta_i*sin_theta_i);

  const T Gamma_s = (n1 * cos_theta_i - n2 * cos_theta_t) / (n1 * cos_theta_i + n2 * cos_theta_t);
  const T Tau_s = 1 + Gamma_s;

  const T Gamma_p = -(n2 * cos_theta_i - n1 * cos_theta_t) / (n2 * cos_theta_i + n1 * cos_theta_t);
  const T Tau_p = n1 / n2 * (1 + Gamma_p);

  return FresnelCoefficients(Gamma_s, Gamma_p, Tau_s, Tau_p);
}

template<typename T>
inline std::pair<Mat3r<T>, Mat3r<T>> polarization_projectors(const Vec3r<T> &H, const Vec3r<T> &norm) {

  Vec3r<T> e_s = normalize(cross(H, norm));
  if (blaze::norm(e_s) <= 0.00001) {
    Vec3r<T> H2;
    for (size_t i = 0; i < 3; i++) {
      H2 = H;
      H2[i] += 0.0001;
      e_s = cross(H2, norm);
      if (blaze::norm(e_s) >= 0.00001)
        break;
    }
  }
  if (dot(H, norm) > 0.)
    e_s *= -1.;

  e_s = normalize(e_s);
  const Vec3r<T> &e_p = normalize(cross(e_s, H));

  const Mat3r<T> &P_s = e_s * trans(e_s);
  const Mat3r<T> &P_p = e_p * trans(e_p);

  return std::make_pair(P_s, P_p);
}

template<typename T>
inline std::pair<Mat3r<T>, Mat3r<T>> polarization_change_tensor(const Vec3r<T> &H, const Vec3r<T> &norm, const T n1, const T n2) {
  const FresnelCoefficients fresnel = fresnel_vector(H, norm, n1, n2);

  const auto [P_s, P_p] = polarization_projectors(H, norm);

  const Mat3r<T> reflection = fresnel.gamma_s * P_s + fresnel.gamma_p * P_p;
  const Mat3r<T> transmission = fresnel.tau_s * P_s + fresnel.tau_p * P_p;

  return std::make_pair(reflection, transmission);
}

template<typename T>
inline std::pair<Mat3r<T>, Mat3r<T>> magnetic_current_tensor(const Vec3r<T> &H, const Vec3r<T> &norm, const T n1, const T n2) {
  const FresnelCoefficients fresnel = fresnel_vector(H, norm, n1, n2);

  const auto [P_s, P_p] = polarization_projectors(H, norm);

  const Mat3r<T> reflection = (1. + fresnel.gamma_s) * P_s + (1. + fresnel.gamma_p) * P_p;
  const Mat3r<T> transmission = fresnel.tau_s * P_s + fresnel.tau_p * P_p;

  return std::make_pair(reflection, transmission);
}

template<typename T>
inline std::pair<Mat3r<T>, Mat3r<T>> electric_current_tensor(const Vec3r<T> &H, const Vec3r<T> &norm, const T n1, const T n2) {
  const FresnelCoefficients fresnel = fresnel_vector(H, norm, n1, n2);

  const auto [P_s, P_p] = polarization_projectors(H, norm);

  const Mat3r<T> reflection = (1. - fresnel.gamma_s) * P_s + (1. - fresnel.gamma_p) * P_p;
  const Mat3r<T> transmission = fresnel.tau_s * P_s + fresnel.tau_p * P_p;

  return std::make_pair(reflection, transmission);
}

// returns true if occluder between from and to.
template<typename T>
inline bool check_for_occluder(const Vec3r<T> &from, const Vec3r<T> &to, const double pad, const Scene<T> &scene, const VectorEMRTPOConfig<T> &config,
                               blazert::RayHit<T> &rayhit) {

  static const T ray_eps = T(config.min_distance);
  const Vec3r<T> dir{to - from};
  const T dist = length(dir);
  const blazert::Ray<T> shadow_ray{from, dir, ray_eps, dist - ray_eps - pad, blazert::Ray<T>::CullBackFace::no, blazert::Ray<T>::AnyHit::no};

  return intersect1(scene, shadow_ray, rayhit);
}


int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Simulation file missing (json)" << std::endl;
    return 1;
  }

  auto config = VectorEMRTPOConfig<double>(argv[1]);

  std::cout << config << std::endl;

  auto vertices = read_2d_array<blazert::Vec3rList<double>, double>(config.points_fname);
  auto triangles = read_2d_array<blazert::Vec3iList, uint32_t>(config.tris_fname);
  auto ids = read_2d_array<Vec4iList, uint32_t>(config.ids_fname);
  auto materials = read_2d_array<blazert::Vec3rList<double>, double>(config.materials_fname);

  blazert::Scene<double> scene;
  [[maybe_unused]] unsigned int geom_id = scene.add_mesh(*vertices, *triangles);
  scene.commit();

  std::vector<VectorEMRTPOData<double>> em_rt_po_data;
  em_rt_po_data.resize(config.shots_per_path * (config.origin_stop_fid - config.origin_start_fid));

  std::srand(config.seed);

  std::cout << "Starting simulation:" << std::endl;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (size_t m = config.origin_start_fid; m < config.origin_stop_fid; m++) {
    const blazert::Vec3ui tri = (*triangles)[m];
    const blazert::Vec3r<double> p0 = (*vertices)[tri[0]];
    const blazert::Vec3r<double> p1 = (*vertices)[tri[1]];
    const blazert::Vec3r<double> p2 = (*vertices)[tri[2]];

    for (size_t s = 0; s < config.shots_per_path; s++) {

      size_t layer = (*ids)[m][0];// 0 -> above layer, 1 -> below layer
      //std::cout << "layer = " << layer << "\n";
      const double u = uniform(0., 1.);
      const double v = uniform(0., 1.);

      /* TODO: We should think about letting origin start at config.origin and direction is then defined by point on source mesh
       *      This would be beneficial for several reasons:
       *        - we would automatically include the path to the origin sphere it self
       *        - material changes within the sphere would also be included
       *        - the "hit" at the origin sphere itself would be caught by checking layers/materials
       */
      blazert::Vec3r<double> origin = (1. - u - v) * p0 + u * p1 + v * p2;//(config.origin);
      blazert::Vec3r<double> direction = origin - config.origin;
      // norm(direction) is the length offset accounting for the sphere
      VectorEMRTPOData<double> em_rt_po_path(m, s, u, v, origin, 0., true, true);

      size_t bounces = 0;
      size_t kind = 0;
      size_t last_kind = 0;
      double attenuation = 1.;
      bool inside_source = false;

      while (true) {

        bounces += 1;

        const blazert::Ray<double> ray{origin, direction, config.min_distance};
        blazert::RayHit<double> rayhit;

        if (!intersect1(scene, ray, rayhit)) {
          break;
        }

        unsigned int fid = rayhit.prim_id;
        // TODO: change inside_source only if actually leaving or entering the source ...
        if ((fid >= config.origin_start_fid) and (fid < config.origin_stop_fid)) {
          // TODO: if inside -> outside and vize versa
          if (inside_source) {
            inside_source = false;
          } else {
            inside_source = true;
          }
        }

        bool inside = (dot(rayhit.normal, ray.direction) > 0.);

        const Vec3r<double> normal = rayhit.normal;

        // TODO: double check this
        const size_t from_medium = inside ? (*ids)[fid][3] : (*ids)[fid][2];
        const size_t to_medium = inside ? (*ids)[fid][2] : (*ids)[fid][3];
        const size_t from_layer = inside ? (*ids)[fid][1] : (*ids)[fid][0];
        const size_t to_layer = inside ? (*ids)[fid][0] : (*ids)[fid][1];

        const double from_ior = (*materials)[from_medium][0];
        const double to_ior = (*materials)[to_medium][0];
        const double from_alpha = (*materials)[from_medium][1];

        // The sourcing is done with from diffusivity. This enables different sourcing behaviors depending
        // on the direction of travel, e.g. for surface reflections or the return wave.
        [[maybe_unused]] const double to_diffusivity = (*materials)[to_medium][2];
        const double from_diffusivity = (*materials)[from_medium][2];

        attenuation *= exp(-from_alpha * rayhit.hit_distance);
        origin += ray.direction * rayhit.hit_distance;// Add a small epsilon (0.1 mm) ...

        // In this case the ray does not change, because this is considered to be unchanged material
        if (((*ids)[fid][2] == (*ids)[fid][3]) or (layer > from_layer)) {
          em_rt_po_path.add_rt_segment(origin, rayhit.hit_distance, from_ior, 1., blaze::IdentityMatrix<double>(3UL),
                                       last_kind);
          continue;
        }// Just continue the ray if the medium is not changed

        RayHit<double> occluder_rayhit;
        if ((!check_for_occluder(origin, config.target, config.target_sphere_radius, scene, config, occluder_rayhit))
            && (uniform(0., 1.) < from_diffusivity)) {

          em_rt_po_path.add_rt_segment(origin, rayhit.hit_distance, from_ior, attenuation,
                                       blaze::IdentityMatrix<double>(3UL), last_kind);

          // find endpoint_id, u, v and save in PO data
          RayHit<double> endpoint_rayhit;
          const double ray_eps = config.min_distance;
          const Vec3r<double> dir{config.target - origin};
          const double dist = blaze::norm(dir);
          const blazert::Ray<double> shadow_ray{
              origin, dir, ray_eps, dist, blazert::Ray<double>::CullBackFace::no, blazert::Ray<double>::AnyHit::no};

          [[maybe_unused]] auto test_hit = intersect1(scene, shadow_ray, endpoint_rayhit);
          em_rt_po_path.finalize(endpoint_rayhit.prim_id, endpoint_rayhit.uv[0], endpoint_rayhit.uv[1]);

          // Add PO parameters
          const auto [magnetic_reflection_current, magnetic_transmission_current] =
              magnetic_current_tensor(ray.direction, normal, from_ior, to_ior);
          const auto [electric_reflection_current, electric_transmission_current] =
              electric_current_tensor(ray.direction, normal, from_ior, to_ior);

          const double ior = !inside ? from_ior : to_ior;
          const Mat3r<double> M_current_matrix = !inside ? magnetic_reflection_current : magnetic_transmission_current;
          const Mat3r<double> J_current_matrix = !inside ? electric_reflection_current : electric_transmission_current;

          em_rt_po_path.add_po_data(ior, dist, origin, normalize(dir), normalize(ray.direction), normal,
                                    M_current_matrix, J_current_matrix);
          break;
        }

        // decide if we want to reflect or refract
        const double next = uniform(0., 1.);
        if ((next < 0.5)
            or ((from_ior > to_ior) and (acos(abs(dot(ray.direction, normal))) >= asin(to_ior / from_ior)))) {
          // Specular Reflect
          kind = 2;
          direction = reflect(ray.direction, normal);// Specular reflect in this case.
        } else {
          direction = refract(ray.direction, normal, from_ior, to_ior, fid);
          layer = to_layer;//(*ids)[fid][0];
          kind = 1;
        }

        // transmit if: new kind is refracted
        // reflect if: new kind is reflected
        const auto [reflection_matrix, transmission_matrix] =
            polarization_change_tensor(ray.direction, normal, from_ior, to_ior);
        const Mat3r<double> pol_matrix = (kind == 1) ? transmission_matrix : reflection_matrix;
        em_rt_po_path.add_rt_segment(origin, rayhit.hit_distance, from_ior, attenuation, pol_matrix, last_kind);
        last_kind = kind;

        // stop criteria
        if ((bounces > config.max_bounces) or (em_rt_po_path.attenuation < config.max_attenuation)
            or (em_rt_po_path.ray_tracing_propagation_length > config.max_propagation_length)) {
          break;
        }

        if (bounces > config.min_bounces) {
          if (uniform(0., 1.) < config.termination_probability) {
            break;
          }
        }
      }
      em_rt_po_data[s + config.shots_per_path * (m - config.origin_start_fid)] = em_rt_po_path;
    }
  }
    //progress_bar((m - config.detector_start_fid) + 1, config.detector_stop_fid - config.detector_start_fid);


  std::ofstream data_outfile;

  if (config.binary_output) {

    const std::string filename = config.data_output_fname  + ".core";
    data_outfile.open(filename, std::ios::trunc | std::ios::binary);
    if (!data_outfile) {
      std::cerr << "Output files for rays and po source could not be written!\n";
      return -1;
    }

    VectorEMRTPOHeader data_header;
    data_header.write(data_outfile);

    size_t n_good_rays = 0;
    for (auto &it : em_rt_po_data) {
      if ((config.output_all_rays) or (it.good_source)) {
        it.write(data_outfile);
        n_good_rays++;
      }
    }

    data_outfile.write(reinterpret_cast<char *>(&n_good_rays), sizeof(n_good_rays));

  } else {
    const std::string filename = config.data_output_fname + ".json";
    data_outfile.open(filename, std::ios::trunc);
    if ((!data_outfile)) {
      std::cerr << "Output files for rays and po source could not be written!\n";
      return -1;
    }
    data_outfile << "[" << std::endl;

    for (auto &it : em_rt_po_data) {
      if ((config.output_all_rays) or (it.good_source)) {
        data_outfile << it;
        if (static_cast<unsigned long>(&it - &*(em_rt_po_data.begin())) < em_rt_po_data.size() - 1) {
          data_outfile << ", " << std::endl;
        }
      }
    }
    data_outfile << "]" << std::endl;
  }

  data_outfile.close();

  std::cout << "Done." << std::endl;
  return 0;
}