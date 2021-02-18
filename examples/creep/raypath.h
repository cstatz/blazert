#pragma once
#ifndef EM_TRACER_RAYPATH_H
#define EM_TRACER_RAYPATH_H

#include <blazert/blazert.h>
#include <cmath>
#include <complex>
#include <fstream>
#include <iostream>
#include <vector>

using namespace blazert;

namespace emrt {
template<typename T>
class RayPath {
public:
  size_t origin_id = -1;
  size_t endpoint_id = -1;// -1 is no hit
  size_t shot = -1;
  T origin_u{}, origin_v{}, endpoint_u{}, endpoint_v{};
  T propagation_length{}, geometric_length{};
  T attenuation{};
  Vec3r<T> origin;
  bool store_endpoints{}, store_kinds{};
  std::vector<Vec3r<T>> endpoints;
  std::vector<size_t> kinds;// {0: initial, 1: refract, 2: reflect_specular, 3: reflect_diffuse}
  std::vector<T> iors;
  std::vector<T> attenuations;

  RayPath() = default;
  [[maybe_unused]] RayPath(size_t origin_id_, size_t shot_, T origin_u_, T origin_v_, const Vec3r<T> &origin_,
                           const T length_offset, bool store_endpoints_ = false, bool store_kinds_ = false)
      : origin_id(origin_id_), endpoint_id(-1), shot(shot_), origin_u(origin_u_), origin_v(origin_v_), endpoint_u(0.),
        endpoint_v(0.), propagation_length(length_offset), geometric_length(length_offset), attenuation(1.),
        origin(origin_), store_endpoints(store_endpoints_), store_kinds(store_kinds_){};

  void add_segment(const Vec3r<T> &endpoint, T local_length, T local_ior, T local_attenuation,
                   const size_t segment_kind = -1) {
    if (store_endpoints) {
      endpoints.emplace_back(endpoint);
      iors.emplace_back(local_ior);
      attenuations.emplace_back(local_attenuation);
    }

    if (store_kinds)
      kinds.emplace_back(segment_kind);

    geometric_length += local_length;
    propagation_length += local_length * local_ior;
    attenuation *= local_attenuation;
  };

  void finalize(size_t endpoint_id_, T u, T v) {
    endpoint_id = endpoint_id_;
    endpoint_u = u;
    endpoint_v = v;
  };

  void read(std::ifstream &f) {
    f.read((char *) (&origin_id), sizeof(origin_id));
    f.read((char *) (&endpoint_id), sizeof(endpoint_id));
    f.read((char *) (&shot), sizeof(shot));
    f.read((char *) (&origin_u), sizeof(origin_u));
    f.read((char *) (&origin_v), sizeof(origin_v));
    f.read((char *) (&propagation_length), sizeof(propagation_length));
    f.read((char *) (&geometric_length), sizeof(geometric_length));
    f.read((char *) (&attenuation), sizeof(attenuation));
    f.read((char *) (&origin[0]), sizeof(origin[0]));
    f.read((char *) (&origin[1]), sizeof(origin[1]));
    f.read((char *) (&origin[2]), sizeof(origin[2]));

    size_t n_endpoints = endpoints.size();
    f.read((char *) (&n_endpoints), sizeof(n_endpoints));

    for (size_t i = 0; i < n_endpoints; i++) {
      blazert::Vec3r<T> x{};
      f.read((char *) (&x[0]), sizeof(x[0]));
      f.read((char *) (&x[1]), sizeof(x[1]));
      f.read((char *) (&x[2]), sizeof(x[2]));
      endpoints.emplace_back(x);
    }

    size_t n_kinds = kinds.size();
    f.read((char *) (&n_kinds), sizeof(n_kinds));

    for (size_t i = 0; i < n_kinds; i++) {
      uint8_t kind;
      f.read((char *) (&kind), sizeof(kind));
      kinds.emplace_back(kind);
    }

    size_t n_iors = iors.size();
    f.read((char *) (&n_iors), sizeof(n_iors));

    for (size_t i = 0; i < n_iors; i++) {
      T ior;
      f.read((char *) (&ior), sizeof(ior));
      iors.emplace_back(ior);
    }

    size_t n_attenuations = attenuations.size();
    f.read((char *) (&n_attenuations), sizeof(n_attenuations));

    for (size_t i = 0; i < n_attenuations; i++) {
      T att;
      f.read((char *) (&att), sizeof(att));
      attenuations.emplace_back(att);
    }
  }

  void write(std::ofstream &f) const {
    f.write((char *) (&origin_id), sizeof(origin_id));
    f.write((char *) (&endpoint_id), sizeof(endpoint_id));
    f.write((char *) (&shot), sizeof(shot));
    f.write((char *) (&origin_u), sizeof(origin_u));
    f.write((char *) (&origin_v), sizeof(origin_v));
    f.write((char *) (&propagation_length), sizeof(propagation_length));
    f.write((char *) (&geometric_length), sizeof(geometric_length));
    f.write((char *) (&attenuation), sizeof(attenuation));
    f.write((char *) (&origin[0]), sizeof(origin[0]));
    f.write((char *) (&origin[1]), sizeof(origin[1]));
    f.write((char *) (&origin[2]), sizeof(origin[2]));

    size_t n_endpoints = endpoints.size();
    f.write((char *) (&n_endpoints), sizeof(n_endpoints));

    for (auto &it : endpoints) {
      f.write((char *) (&it[0]), sizeof(it[0]));
      f.write((char *) (&it[1]), sizeof(it[1]));
      f.write((char *) (&it[2]), sizeof(it[2]));
    }

    size_t n_kinds = kinds.size();
    f.write((char *) (&n_kinds), sizeof(n_kinds));

    for (auto &it : kinds) {
      uint8_t kind = it;
      f.write((char *) (&kind), sizeof(kind));
    }

    size_t n_iors = iors.size();
    f.write((char *) (&n_iors), sizeof(n_iors));

    for (auto &it : iors) {
      T ior = it;
      f.write((char *) (&ior), sizeof(ior));
    }

    size_t n_attenuations = attenuations.size();
    f.write((char *) (&n_attenuations), sizeof(n_attenuations));

    for (auto &it : attenuations) {
      T att = it;
      f.write((char *) (&att), sizeof(att));
    }
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const RayPath<T> &p) {

  stream << "{\n";
  stream << R"(  "origin_id": )" << p.origin_id << ",\n";
  stream << R"(  "endpoint_id": )" << p.endpoint_id << ",\n";
  stream << R"(  "shot": )" << p.shot << ",\n";
  stream << R"(  "origin": [ )" << p.origin[0] << ", " << p.origin[1] << ", " << p.origin[2] << "],\n";
  stream << R"(  "propagation_length": )" << p.propagation_length << ",\n";
  stream << R"(  "geometric_length": )" << p.propagation_length << ",\n";

  stream << R"(  "endpoints": [ )"
         << "\n";
  for (auto &it : p.endpoints) {
    stream << "[" << it[0] << ", " << it[1] << ", " << it[2] << "]";

    if (&it - &*(p.endpoints.begin()) < p.endpoints.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }
  stream << ",\n";

  stream << R"("kinds": [ )"
         << "\n";
  for (auto &it : p.kinds) {
    stream << it;

    if (&it - &*(p.kinds.begin()) < p.kinds.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }
  stream << ",\n";

  stream << R"("iors": [ )"
         << "\n";
  for (auto &it : p.iors) {
    stream << it;

    if (&it - &*(p.iors.begin()) < p.iors.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
  }
  stream << ",\n";

  stream << R"("attenuations": [ )"
         << "\n";
  for (auto &it : p.attenuations) {
    stream << it;

    if (&it - &*(p.attenuations.begin()) < p.attenuations.size() - 1) {
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

#endif//EM_TRACER_RAYPATH_H
