#pragma once
#ifndef EMRTPO_TRACER_CONFIG_H
#define EMRTPO_TRACER_CONFIG_H

#include <blazert/blazert.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using json = nlohmann::json;

template<typename T>
class VectorEMRTPOConfig {
public:
  blazert::Vec3r<T> origin;
  blazert::Vec3r<T> target;
  T origin_sphere_radius, target_sphere_radius, termination_probability, max_attenuation, max_propagation_length,
      min_distance;
  size_t origin_start_fid{}, origin_stop_fid{}, target_start_fid{}, target_stop_fid{}, shots_per_path{}, max_bounces{},
      min_bounces{};

  std::string points_fname, tris_fname, ids_fname, materials_fname, data_output_fname;
  bool binary_output{}, output_all_rays{};
  unsigned int seed{};

  VectorEMRTPOConfig() = default;

  explicit VectorEMRTPOConfig(const char *fname) {
    auto json_file = std::ifstream(fname);

    if (json_file) {
      json config_data = json::parse(json_file);
      points_fname = config_data["points_file"];
      tris_fname = config_data["tris_file"];
      ids_fname = config_data["ids_file"];
      materials_fname = config_data["materials_file"];
      data_output_fname = config_data["results_file"];
      origin_sphere_radius = config_data["origin_sphere_radius"];
      origin_start_fid = config_data["origin_sphere_range"][0];
      origin_stop_fid = config_data["origin_sphere_range"][1];
      target_start_fid = config_data["target_sphere_range"][0];
      target_stop_fid = config_data["target_sphere_range"][1];
      target_sphere_radius = config_data["target_sphere_radius"];
      shots_per_path = config_data["shots_per_path"];
      max_bounces = config_data["max_bounces"];
      min_bounces = config_data["min_bounces"];
      termination_probability = config_data["termination_probability"];
      max_attenuation = config_data["max_attenuation"];
      max_propagation_length = config_data["max_propagation_length"];
      min_distance = config_data["min_distance"];
      binary_output = config_data["binary_output"];
      output_all_rays = config_data["output_all_rays"];
      seed = config_data["seed"];

      for (size_t i = 0; i < config_data["origin"].size(); i++) {
        origin[i] = config_data["origin"][i];
      }

      for (size_t j = 0; j < config_data["target"].size(); j++) {
        target[j] = config_data["target"][j];
      }

    } else {
      std::cout << "ERROR: Could not load VectorEMRTPOConfig from " << fname << "\n";
    }
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const VectorEMRTPOConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "data_results_file": )" << config.data_output_fname << (config.binary_output ? ".core" : ".json")
         << ",\n";
  stream << R"(  "points_file": )" << config.points_fname << ",\n";
  stream << R"(  "tris_file": )" << config.tris_fname << ",\n";
  stream << R"(  "ids_file": )" << config.ids_fname << ",\n";
  stream << R"(  "materials_file": )" << config.materials_fname << ",\n";
  stream << R"(  "origin": [ )" << config.origin[0] << ", " << config.origin[1] << ", " << config.origin[2] << "],\n";
  stream << R"(  "origin_sphere_radius": )" << config.origin_sphere_radius << ",\n";
  stream << R"(  "origin_sphere_range": [ )" << config.origin_start_fid << ", " << config.origin_stop_fid << "],\n";
  stream << R"(  "target": [ )" << config.target[0] << ", " << config.target[1] << ", " << config.target[2] << "],\n";
  stream << R"(  "target_sphere_radius": )" << config.target_sphere_radius << ",\n";
  stream << R"(  "target_sphere_range": [)" << config.target_start_fid << ", " << config.target_stop_fid << "],\n";
  stream << R"(  "shots_per_path": )" << config.shots_per_path << ",\n";
  stream << R"(  "max_bounces": )" << config.max_bounces << ",\n";
  stream << R"(  "min_bounces": )" << config.min_bounces << ",\n";
  stream << R"(  "termination_probability": )" << config.termination_probability << ",\n";
  stream << R"(  "max_attenuation": )" << config.max_attenuation << ",\n";
  stream << R"(  "min_distance": )" << config.min_distance << ",\n";
  stream << R"(  "max_propagation_length": )" << config.max_propagation_length << ",\n";
  stream << R"(  "binary_output": )" << config.binary_output << ",\n";
  stream << R"(  "output_all_rays": )" << config.output_all_rays << "\n";
  stream << R"(  "seed": )" << config.seed << "\n";

  stream << "}";
  return stream;
}

#endif//EMRTPO_TRACER_CONFIG_H
