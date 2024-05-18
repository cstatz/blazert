#pragma once
#ifndef EMRTPO_DATA_H
#define EMRTPO_DATA_H

#include "blazert/blazert.h"
#include <fstream>
#include <iostream>
#include <vector>

namespace emrt {

template<typename T>
class VectorEMRTPOData {
public:
  bool good_source = false;

  size_t origin_fid = static_cast<size_t>(-1);
  size_t endpoint_fid = static_cast<size_t>(-1);

  size_t shot = static_cast<size_t>(-1);

  T origin_u{}, origin_v{}, endpoint_u{}, endpoint_v{};

  blazert::Vec3r<T> origin{};
  blazert::Vec3r<T> target{};

  // RT data
  T attenuation{};
  T ray_tracing_geometric_length{}, ray_tracing_propagation_length{};

  // PO data
  T physical_optics_geometric_length{}, physical_optics_propagation_length{};
  T po_ior{};
  blazert::Vec3r<T> po_source_origin{};
  blazert::Vec3r<T> po_direction{};
  blazert::Vec3r<T> po_incidence{};
  blazert::Vec3r<T> po_surface_normal{};
  blazert::Mat3r<T> magnetic_current_projector{}, electric_current_projector{};

  // general EM data
  blazert::Mat3r<T> polarization_transformation{};

  // visualization & debug data
  bool store_endpoints{}, store_kinds{};
  std::vector<blazert::Vec3r<T>> endpoints;
  std::vector<size_t> kinds;// {0: initial, 1: refract, 2: reflect_specular, 3: reflect_diffuse}
  std::vector<T> iors;
  std::vector<T> attenuations;

  VectorEMRTPOData() = default;
  VectorEMRTPOData(const VectorEMRTPOData &) = default;

  [[maybe_unused]] VectorEMRTPOData(size_t origin_id_, size_t shot_, T origin_u_, T origin_v_,
                                    const blazert::Vec3r<T> &origin_, const T length_offset,
                                    bool store_endpoints_ = false, bool store_kinds_ = false)
      : origin_fid(origin_id_), endpoint_fid(static_cast<size_t>(-1)), shot(shot_), origin_u(origin_u_),
        origin_v(origin_v_), endpoint_u(0.), endpoint_v(0.), origin(origin_), attenuation(1.),
        ray_tracing_geometric_length(length_offset), ray_tracing_propagation_length(length_offset),
        polarization_transformation(blaze::IdentityMatrix<T>(3UL)), store_endpoints(store_endpoints_),
        store_kinds(store_kinds_){};

  void add_rt_segment(const blazert::Vec3r<T> &endpoint, const T local_length, const T local_ior,
                      const T local_attenuation, const blazert::Mat3r<T> local_polarization,
                      const size_t segment_kind = static_cast<size_t>(-1)) {

    if (store_endpoints) {
      endpoints.emplace_back(endpoint);
      iors.emplace_back(local_ior);
      attenuations.emplace_back(local_attenuation);
    }

    if (store_kinds)
      kinds.emplace_back(segment_kind);

    ray_tracing_geometric_length += local_length;
    ray_tracing_propagation_length += local_length * local_ior;
    attenuation *= local_attenuation;
    polarization_transformation *= local_polarization;
  };

  void add_po_data(const T ior_, const T length_, const blazert::Vec3r<T> &source_origin_,
                   const blazert::Vec3r<T> &direction_, const blazert::Vec3r<T> &incidence_,
                   const blazert::Vec3r<T> &normal_, const blazert::Mat3r<T> &magnetic_current_,
                   const blazert::Mat3r<T> &electric_current_) {
    po_ior = ior_;
    physical_optics_geometric_length = length_;
    physical_optics_propagation_length = ior_ * length_;
    po_source_origin = source_origin_;
    po_direction = direction_;
    po_incidence = incidence_;
    po_surface_normal = normal_;
    magnetic_current_projector = magnetic_current_;
    electric_current_projector = electric_current_;
  };

  void finalize(size_t endpoint_id_, T u, T v) {
    endpoint_fid = endpoint_id_;
    endpoint_u = u;
    endpoint_v = v;
    good_source = true;
  };

  void read(std::ifstream &f) {
    f.read(reinterpret_cast<char *>(&good_source), sizeof(good_source));
    f.read(reinterpret_cast<char *>(&origin_fid), sizeof(origin_fid));
    f.read(reinterpret_cast<char *>(&endpoint_fid), sizeof(endpoint_fid));
    f.read(reinterpret_cast<char *>(&shot), sizeof(shot));
    f.read(reinterpret_cast<char *>(&origin_u), sizeof(origin_u));
    f.read(reinterpret_cast<char *>(&origin_v), sizeof(origin_v));
    f.read(reinterpret_cast<char *>(&endpoint_u), sizeof(endpoint_u));
    f.read(reinterpret_cast<char *>(&endpoint_v), sizeof(endpoint_v));
    f.read(reinterpret_cast<char *>(&origin[0]), sizeof(origin[0]));
    f.read(reinterpret_cast<char *>(&origin[1]), sizeof(origin[1]));
    f.read(reinterpret_cast<char *>(&origin[2]), sizeof(origin[2]));
    f.read(reinterpret_cast<char *>(&target[0]), sizeof(target[0]));
    f.read(reinterpret_cast<char *>(&target[1]), sizeof(target[1]));
    f.read(reinterpret_cast<char *>(&target[2]), sizeof(target[2]));
    f.read(reinterpret_cast<char *>(&attenuation), sizeof(attenuation));
    f.read(reinterpret_cast<char *>(&ray_tracing_geometric_length), sizeof(ray_tracing_geometric_length));
    f.read(reinterpret_cast<char *>(&ray_tracing_propagation_length), sizeof(ray_tracing_propagation_length));
    f.read(reinterpret_cast<char *>(&physical_optics_geometric_length), sizeof(physical_optics_geometric_length));
    f.read(reinterpret_cast<char *>(&physical_optics_propagation_length), sizeof(physical_optics_propagation_length));
    f.read(reinterpret_cast<char *>(&po_ior), sizeof(po_ior));
    f.read(reinterpret_cast<char *>(&po_source_origin[0]), sizeof(po_source_origin[0]));
    f.read(reinterpret_cast<char *>(&po_source_origin[1]), sizeof(po_source_origin[1]));
    f.read(reinterpret_cast<char *>(&po_source_origin[2]), sizeof(po_source_origin[2]));
    f.read(reinterpret_cast<char *>(&po_direction[0]), sizeof(po_direction[0]));
    f.read(reinterpret_cast<char *>(&po_direction[1]), sizeof(po_direction[1]));
    f.read(reinterpret_cast<char *>(&po_direction[2]), sizeof(po_direction[2]));
    f.read(reinterpret_cast<char *>(&po_incidence[0]), sizeof(po_incidence[0]));
    f.read(reinterpret_cast<char *>(&po_incidence[1]), sizeof(po_incidence[1]));
    f.read(reinterpret_cast<char *>(&po_incidence[2]), sizeof(po_incidence[2]));
    f.read(reinterpret_cast<char *>(&po_surface_normal[0]), sizeof(po_surface_normal[0]));
    f.read(reinterpret_cast<char *>(&po_surface_normal[1]), sizeof(po_surface_normal[1]));
    f.read(reinterpret_cast<char *>(&po_surface_normal[2]), sizeof(po_surface_normal[2]));
    f.read(reinterpret_cast<char *>(&magnetic_current_projector), sizeof(magnetic_current_projector));
    f.read(reinterpret_cast<char *>(&electric_current_projector), sizeof(electric_current_projector));
    f.read(reinterpret_cast<char *>(&polarization_transformation), sizeof(polarization_transformation));

    size_t n_endpoints = endpoints.size();
    f.read(reinterpret_cast<char *>(&n_endpoints), sizeof(n_endpoints));

    for (size_t i = 0; i < n_endpoints; i++) {
      blazert::Vec3r<T> x{};
      f.read(reinterpret_cast<char *>(&x[0]), sizeof(x[0]));
      f.read(reinterpret_cast<char *>(&x[1]), sizeof(x[1]));
      f.read(reinterpret_cast<char *>(&x[2]), sizeof(x[2]));
      endpoints.emplace_back(x);
    }

    size_t n_kinds = kinds.size();
    f.read(reinterpret_cast<char *>(&n_kinds), sizeof(n_kinds));

    for (size_t i = 0; i < n_kinds; i++) {
      size_t kind;
      f.read(reinterpret_cast<char *>(&kind), sizeof(kind));
      kinds.emplace_back(kind);
    }

    size_t n_iors = iors.size();
    f.read(reinterpret_cast<char *>(&n_iors), sizeof(n_iors));

    for (size_t i = 0; i < n_iors; i++) {
      T ior;
      f.read(reinterpret_cast<char *>(&ior), sizeof(ior));
      iors.emplace_back(ior);
    }

    size_t n_attenuations = attenuations.size();
    f.read(reinterpret_cast<char *>(&n_attenuations), sizeof(n_attenuations));

    for (size_t i = 0; i < n_attenuations; i++) {
      T att;
      f.read(reinterpret_cast<char *>(&att), sizeof(att));
      attenuations.emplace_back(att);
    }
  };

  void write(std::ofstream &f) const {

    f.write(reinterpret_cast<const char *>(&good_source), sizeof(good_source));
    f.write(reinterpret_cast<const char *>(&origin_fid), sizeof(origin_fid));
    f.write(reinterpret_cast<const char *>(&endpoint_fid), sizeof(endpoint_fid));
    f.write(reinterpret_cast<const char *>(&shot), sizeof(shot));
    f.write(reinterpret_cast<const char *>(&origin_u), sizeof(origin_u));
    f.write(reinterpret_cast<const char *>(&origin_v), sizeof(origin_v));
    f.write(reinterpret_cast<const char *>(&endpoint_u), sizeof(endpoint_u));
    f.write(reinterpret_cast<const char *>(&endpoint_v), sizeof(endpoint_v));
    f.write(reinterpret_cast<const char *>(&origin[0]), sizeof(origin[0]));
    f.write(reinterpret_cast<const char *>(&origin[1]), sizeof(origin[1]));
    f.write(reinterpret_cast<const char *>(&origin[2]), sizeof(origin[2]));
    f.write(reinterpret_cast<const char *>(&target[0]), sizeof(target[0]));
    f.write(reinterpret_cast<const char *>(&target[1]), sizeof(target[1]));
    f.write(reinterpret_cast<const char *>(&target[2]), sizeof(target[2]));
    f.write(reinterpret_cast<const char *>(&attenuation), sizeof(attenuation));
    f.write(reinterpret_cast<const char *>(&ray_tracing_geometric_length), sizeof(ray_tracing_geometric_length));
    f.write(reinterpret_cast<const char *>(&ray_tracing_propagation_length), sizeof(ray_tracing_propagation_length));
    f.write(reinterpret_cast<const char *>(&physical_optics_geometric_length),
            sizeof(physical_optics_geometric_length));
    f.write(reinterpret_cast<const char *>(&physical_optics_propagation_length),
            sizeof(physical_optics_propagation_length));
    f.write(reinterpret_cast<const char *>(&po_ior), sizeof(po_ior));
    f.write(reinterpret_cast<const char *>(&po_source_origin[0]), sizeof(po_source_origin[0]));
    f.write(reinterpret_cast<const char *>(&po_source_origin[1]), sizeof(po_source_origin[1]));
    f.write(reinterpret_cast<const char *>(&po_source_origin[2]), sizeof(po_source_origin[2]));
    f.write(reinterpret_cast<const char *>(&po_direction[0]), sizeof(po_direction[0]));
    f.write(reinterpret_cast<const char *>(&po_direction[1]), sizeof(po_direction[1]));
    f.write(reinterpret_cast<const char *>(&po_direction[2]), sizeof(po_direction[2]));
    f.write(reinterpret_cast<const char *>(&po_incidence[0]), sizeof(po_incidence[0]));
    f.write(reinterpret_cast<const char *>(&po_incidence[1]), sizeof(po_incidence[1]));
    f.write(reinterpret_cast<const char *>(&po_incidence[2]), sizeof(po_incidence[2]));
    f.write(reinterpret_cast<const char *>(&po_surface_normal[0]), sizeof(po_surface_normal[0]));
    f.write(reinterpret_cast<const char *>(&po_surface_normal[1]), sizeof(po_surface_normal[1]));
    f.write(reinterpret_cast<const char *>(&po_surface_normal[2]), sizeof(po_surface_normal[2]));
    f.write(reinterpret_cast<const char *>(&magnetic_current_projector), sizeof(magnetic_current_projector));
    f.write(reinterpret_cast<const char *>(&electric_current_projector), sizeof(electric_current_projector));
    f.write(reinterpret_cast<const char *>(&polarization_transformation), sizeof(polarization_transformation));

    size_t n_endpoints = endpoints.size();
    f.write(reinterpret_cast<const char *>(&n_endpoints), sizeof(n_endpoints));

    for (auto &it : endpoints) {
      f.write(reinterpret_cast<const char *>(&it[0]), sizeof(it[0]));
      f.write(reinterpret_cast<const char *>(&it[1]), sizeof(it[1]));
      f.write(reinterpret_cast<const char *>(&it[2]), sizeof(it[2]));
    }

    size_t n_kinds = kinds.size();

    f.write(reinterpret_cast<const char *>(&n_kinds), sizeof(n_kinds));

    for (auto &it : kinds) {
      size_t kind = it;
      f.write(reinterpret_cast<const char *>(&kind), sizeof(kind));
    }

    size_t n_iors = iors.size();
    f.write(reinterpret_cast<const char *>(&n_iors), sizeof(n_iors));

    for (auto &it : iors) {
      T ior = it;
      f.write(reinterpret_cast<const char *>(&ior), sizeof(ior));
    }

    size_t n_attenuations = attenuations.size();
    f.write(reinterpret_cast<const char *>(&n_attenuations), sizeof(n_attenuations));

    for (auto &it : attenuations) {
      T att = it;
      f.write(reinterpret_cast<const char *>(&att), sizeof(att));
    }
  };
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const VectorEMRTPOData<T> &p) {

  stream << "{\n";
  stream << R"(  "origin_id": )" << p.origin_fid << ",\n";
  stream << R"(  "endpoint_id": )" << p.endpoint_fid << ",\n";
  stream << R"(  "shot": )" << p.shot << ",\n";
  stream << R"(  "origin": [ )" << p.origin[0] << ", " << p.origin[1] << ", " << p.origin[2] << "],\n";
  stream << R"(  "propagation_length": )" << p.ray_tracing_propagation_length << ",\n";
  stream << R"(  "geometric_length": )" << p.ray_tracing_geometric_length << ",\n";

  stream << R"(  "endpoints": [ )" << "\n";
  for (auto &it : p.endpoints) {
    stream << "[" << it[0] << ", " << it[1] << ", " << it[2] << "]";

    if (static_cast<unsigned long>(&it - &*(p.endpoints.begin())) < p.endpoints.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }
  stream << ",\n";

  stream << R"("kinds": [ )" << "\n";
  for (auto &it : p.kinds) {
    stream << it;

    if (static_cast<unsigned long>(&it - &*(p.kinds.begin())) < p.kinds.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }
  stream << ",\n";

  stream << R"("iors": [ )" << "\n";
  for (auto &it : p.iors) {
    stream << it;

    if (static_cast<unsigned long>(&it - &*(p.iors.begin())) < p.iors.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }
  stream << ",\n";

  stream << R"("attenuations": [ )" << "\n";
  for (auto &it : p.attenuations) {
    stream << it;

    if (static_cast<unsigned long>(&it - &*(p.attenuations.begin())) < p.attenuations.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }

  // TODO: Output attenuation ...

  stream << "}";

  return stream;
}

}// namespace emrt

#endif//EMRTPO_DATA_H
