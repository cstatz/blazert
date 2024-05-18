#pragma once
#ifndef EMRTPO_PATTERN_PHYSICS_H
#define EMRTPO_PATTERN_PHYSICS_H

#include "emrtpo_data.h"
#include "emrtpo_data_header.h"
#include "emrtpo_po_config.h"
#include "emrtpo_tracer_config.h"

#include "constants.h"
#include "transforms.h"
#include <cmath>
#include <complex>

using namespace std::complex_literals;
using namespace blazert;
using namespace emrt;

// TODO: inline
inline __attribute__((always_inline)) std::complex<double>
spherical_wave_greens_function(const double propagation_length, const double k) {
  // if the real part in the exponential is 0, then std::polar is supposed to be fast:
  // https://en.cppreference.com/w/cpp/numeric/complex/polar
  return std::exp(-1.i * k * propagation_length) / (4. * pi * propagation_length);
  //return std::polar(1./(4. * pi * propagation_length), -k*propagation_length);
}

inline __attribute__((always_inline)) std::complex<double> plane_wave_greens_function(const double propagation_length,
                                                                                      const double k) {
  // if the real part in the exponential is 0, then std::polar is supposed to be fast:
  // https://en.cppreference.com/w/cpp/numeric/complex/polar
  return std::exp(-1.i * k * propagation_length);
  //return std::polar(1./(4. * pi * propagation_length), -k*propagation_length);
}

template<typename T>
inline __attribute__((always_inline)) Mat3r<T> cross_matrix(const Vec3r<T> &v) {
  const Mat3r<T> tmp{{0, -v[2], v[1]}, {v[2], 0, -v[0]}, {-v[1], v[0], 0}};
  return tmp;
}

// TODO: maybe inline, measure!
// TODO: what is really needed here? do we need the whole config?
// Needed is po and raypaths, but not all kinds, endpoints ... this creates an overhead of the adresses of the vectors ...
// origin, start_frequency, frequency_step, n_frequencies ...
// triangles, pattern, vertex_offset?
inline blaze::DynamicVector<std::complex<double>>
pattern_integrator(const emrt::VectorEMRTPOData<double> &em_rt_po_data, const VectorEMRTPOConfig<double> &emrt_config,
                   const VectorEMRTPOResultsConfig<double> &results_config, const blazert::Vec3iList &triangles,
                   const Vec3rList<std::complex<double>> &pattern, const size_t offset_src_vertex,
                   const double start_freq, const double freq_step, const size_t n_frequencies) {

  // TODO: This should ideally fit into 256kB!
  // TODO: This is constant for all paths, compute outside! Ideally pre-compute frequency-vector.
  // attitude of the source/target
  blaze::DynamicVector<std::complex<double>> field_out(n_frequencies, 0);
  const Mat3r<double> &to_global = results_config.attitude_origin;
  const Mat3r<double> &to_local = trans(to_global);

  const Mat3r<double> &polarization_change = em_rt_po_data.polarization_transformation;
  const Mat3r<double> &magnetic_current_projector = em_rt_po_data.magnetic_current_projector;
  const Mat3r<double> &electric_current_projector = em_rt_po_data.electric_current_projector;
  const Vec3r<double> &first_direction = to_local * (em_rt_po_data.origin - emrt_config.origin);
  const Vec3r<double> &origin_sph = cart2sph(normalize(first_direction));
  const Mat3r<double> &origin_sph2cart = sph2cart_matrix(origin_sph[0], origin_sph[1]);
  const Vec3r<double> &target = normalize(to_local * -em_rt_po_data.po_direction);
  const Vec3r<double> &target_sph = cart2sph(normalize(target));
  const Mat3r<double> &target_sph2cart = sph2cart_matrix(target_sph[0], target_sph[1]);
  const Vec3r<double> &sph_from = cart2sph(normalize(target));
  const Vec3r<double> &sph_to = cart2sph(-normalize(target));
  const Mat3r<double> &flip_incidence =
      sph2cart_matrix(sph_to[0], sph_to[1]) * cart2sph_matrix(sph_from[0], sph_from[1]);

  // TODO: This should  include the origin sphere radius as well... this is only sphere to surface..
  const double propagation_length_to_last_surface =
      blaze::length(first_direction) + em_rt_po_data.ray_tracing_propagation_length;
  const double PO_length = em_rt_po_data.physical_optics_propagation_length;

  // return if ambiguous
  if ((results_config.assert_unambiguous_range)
      && (propagation_length_to_last_surface + PO_length > speed_of_light / freq_step))
    return field_out;

  // origin_id, endpoint_id, pattern, triangles, offset (how and where to compute)
  std::array<Vec3r<std::complex<double>>, 6 * 512> local_pattern_values;

  const blazert::Vec3ui &tri_src = (triangles)[em_rt_po_data.origin_fid];
  const blazert::Vec3ui &tri_snk = (triangles)[em_rt_po_data.endpoint_fid];

  //TODO: This needs to be checked!!!
  // 6 individual rows to reduce cache misses
  for (size_t f = 0; f < n_frequencies; f++) {
    local_pattern_values[6 * f] = pattern[(tri_src[0] - offset_src_vertex) * n_frequencies + f];
  }
  for (size_t f = 0; f < n_frequencies; f++) {
    local_pattern_values[6 * f + 1] = pattern[(tri_src[1] - offset_src_vertex) * n_frequencies + f];
  }
  for (size_t f = 0; f < n_frequencies; f++) {
    local_pattern_values[6 * f + 2] = pattern[(tri_src[2] - offset_src_vertex) * n_frequencies + f];
  }
  // This only works for multistatic things
  // TODO: fix this somehow
  const size_t temp_offset = 10242;// res 6 => 40962, res 5 => 10242;
  for (size_t f = 0; f < n_frequencies; f++) {
    local_pattern_values[6 * f + 3] = pattern[(tri_snk[0] - offset_src_vertex - temp_offset) * n_frequencies + f];
  }
  for (size_t f = 0; f < n_frequencies; f++) {
    local_pattern_values[6 * f + 4] = pattern[(tri_snk[1] - offset_src_vertex - temp_offset) * n_frequencies + f];
  }
  for (size_t f = 0; f < n_frequencies; f++) {
    local_pattern_values[6 * f + 5] = pattern[(tri_snk[2] - offset_src_vertex - temp_offset) * n_frequencies + f];
  }
  // 72 kB

  // TODO: Correct order
  const double uo = em_rt_po_data.origin_u;
  const double vo = em_rt_po_data.origin_v;
  const double ue = em_rt_po_data.endpoint_u;
  const double ve = em_rt_po_data.endpoint_v;
  const size_t n_endpoints = em_rt_po_data.endpoints.size();
  const double attenuation = em_rt_po_data.attenuation;
  const Vec3r<double> &normal = em_rt_po_data.po_surface_normal;
  const Vec3r<double> &direction = normalize(to_local * em_rt_po_data.po_direction);//TODO: rotation?
  const Mat3r<double> &normal_cross = cross_matrix(normal);
  const Mat3r<double> &direction_cross = cross_matrix(direction);
  const Vec3r<double> &incidence_direction =
      normalize(to_local * em_rt_po_data.po_incidence);//TODO: this should be rotated as well...
  const Mat3r<double> &incidence_cross = cross_matrix(incidence_direction);
  const double current_ior = em_rt_po_data.iors[n_endpoints - 1];
  const double last_ior = em_rt_po_data.po_ior;

  // TODO: We need to talk about this compensation
  const double two_power_endpoints = 1.;// static_cast<double>(1ULL << (n_endpoints));
  //if(n_endpoints == 1) two_power_endpoints = 1;

  // normalization on the number of rays is done after the integration.
  // TODO: This should be the geometric length, not the propagation length
  const double surface_element = 4. * pi * PO_length * PO_length;
  // NB: HERE ONLY FREQUENCY DEPENDENT COMPUTATIONS! NO MORE PRE-COMPUTATION! AVOID INDIRECTIONS.
  // NB: HERE NO MORE L3-Cache-Misses (L1 is OK.)!
  for (size_t k = 0; k < n_frequencies; ++k) {
    // TODO: This should fit ideally into 16kB!
    // This is the hot path. Here we want no cache-misses (especially down to the RAM!)

    // This is copied en bloque (6*3*16).
    const Vec3r<std::complex<double>> &E0_src = local_pattern_values[k * 6 + 0];
    const Vec3r<std::complex<double>> &E1_src = local_pattern_values[k * 6 + 1];
    const Vec3r<std::complex<double>> &E2_src = local_pattern_values[k * 6 + 2];
    const Vec3r<std::complex<double>> &E0_snk = local_pattern_values[k * 6 + 3];
    const Vec3r<std::complex<double>> &E1_snk = local_pattern_values[k * 6 + 4];
    const Vec3r<std::complex<double>> &E2_snk = local_pattern_values[k * 6 + 5];

    const Vec3r<std::complex<double>> &E_source = ((1. - uo - vo) * E0_src + uo * E1_src + vo * E2_src);
    const Vec3r<std::complex<double>> &E_sink = ((1. - ue - ve) * E0_snk + ue * E1_snk + ve * E2_snk);

    // Ideally, only a subset of data needs to be exchanged.
    const double free_space_wavenumber = twopioc * (start_freq + static_cast<double>(k) * freq_step);
    const Vec3r<std::complex<double>> &receiver_field = to_global * blaze::conj(target_sph2cart * E_sink);
    const Vec3r<std::complex<double>> &transmitter_field = to_global * origin_sph2cart * E_source;
    const Vec3r<std::complex<double>> &surface_field =
        spherical_wave_greens_function(propagation_length_to_last_surface, free_space_wavenumber) * attenuation
        * polarization_change * transmitter_field;

    const Vec3r<std::complex<double>> &magnetic_source_current =
        -normal_cross * magnetic_current_projector * surface_field;
    const Vec3r<std::complex<double>> &surface_H_field =
        current_ior / free_space_wave_impedance * incidence_cross * surface_field;
    const Vec3r<std::complex<double>> &electric_source_current =
        normal_cross * electric_current_projector * surface_H_field;

    // TODO: Not sure if both parts are positive or not..
    const Vec3r<std::complex<double>> &total_source_current = direction_cross * magnetic_source_current
        + direction_cross * direction_cross * free_space_wave_impedance / last_ior * electric_source_current;
    const Vec3r<std::complex<double>> &integrand =
        spherical_wave_greens_function(PO_length, free_space_wavenumber) * total_source_current * surface_element;
    const Vec3r<std::complex<double>> &transmitted_field = -1.i * free_space_wavenumber * integrand;

    // TODO: double(2ULL << n_endpoints) -> Number of endpoints should be limited to maybe 30 or 50 to avoid denormalized numbers if UL or ULL and then double!
    // NB: In 1e9 raypath there is maybe one raypath with 30 bounces ...
    // TODO: It is very unlikely that there will be such path, if this is weak scattering, use different approximation!
    // TODO: Also: Number of bounces limits the number of rays (and vice versa).
    field_out[k] = two_power_endpoints * blaze::trans(flip_incidence * receiver_field) * transmitted_field;
  }
  return field_out;
}

/***
 * Processes a single target position given a source and target location
 * @tparam T
 * @param emrt_config
 * @param config
 * @param target_index
 * @param pattern
 * @param temporary_output_fname
 * @return
 */
template<typename T>
blaze::DynamicVector<std::complex<T>>
process_single_target_position(const VectorEMRTPOConfig<T> &emrt_config, const VectorEMRTPOResultsConfig<T> &config,
                               const Vec3rList<std::complex<T>> &pattern, const std::string &temporary_output_fname) {

  blaze::DynamicVector<std::complex<T>> field(config.n_frequencies, 0.);

  auto triangles = read_2d_array<blazert::Vec3iList, uint32_t>(emrt_config.tris_fname);

  std::ifstream data_is(emrt_config.data_output_fname + ".core", std::ifstream::binary);

  if (data_is) {
    // get length of file:
    data_is.seekg(0, std::ifstream::end);

    size_t length = static_cast<size_t>(data_is.tellg());
    size_t n_good_rays = 0;

    data_is.seekg(static_cast<long>(length - sizeof(n_good_rays)));
    data_is.read(reinterpret_cast<char *>(&n_good_rays), sizeof(n_good_rays));
    data_is.seekg(0, std::ifstream::beg);

    std::cout << "Number of rays: " << n_good_rays << std::endl;

    VectorEMRTPOHeader h;
    h.read(data_is);

    //blaze::min() -> The minimum search over all triangles really slowed the processing down.
    const size_t offset = blaze::min((*triangles)[emrt_config.origin_start_fid]);

    const double start_frequency = config.start_frequency;
    const double stop_frequency = config.stop_frequency;
    const size_t n_frequencies = config.n_frequencies;
    const double freq_step = (stop_frequency - start_frequency) / static_cast<double>(n_frequencies - 1);
    std::cout << "Start processing!\n";

    const size_t n_rays = (emrt_config.origin_stop_fid - emrt_config.origin_start_fid) * emrt_config.shots_per_path;
    for (size_t k = 0; k < n_good_rays; k++) {
      // TODO: Here processing speed is limited by disk-io speed!
      // TODO: This is sequential read, this should in total amount to <10s for 1e6 Raypaths.
      VectorEMRTPOData<double> data;
      data.read(data_is);
      field += std::move(pattern_integrator(data, emrt_config, config, *triangles, pattern, offset, start_frequency,
                                            freq_step, n_frequencies));
    }
    std::cout << "Done Processing\n";
    field /= n_rays;

    cnpy::npy_save(temporary_output_fname, field.data(), {config.n_frequencies}, "w");

    data_is.close();
  }
  return field;
}

#endif//EMRTPO_PATTERN_PHYSICS_H
