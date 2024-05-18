#pragma once
#ifndef EMRTPO_PO_CONFIG_H
#define EMRTPO_PO_CONFIG_H

#include <blazert/blazert.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "emrtpo_tracer_config.h"

using json = nlohmann::json;

template<typename T>
class VectorEMRTPOResultsConfig {
public:
  T start_frequency, stop_frequency;
  size_t n_frequencies;
  blazert::Mat3r<T> attitude_origin;
  blazert::Mat3r<T> attitude_target;
  std::string emrtpo_config;
  std::string output_filename;
  bool assert_unambiguous_range;
  bool clean_temporary_files{false};

  explicit VectorEMRTPOResultsConfig(const char *fname) {

    json config_data = json::parse(std::ifstream(fname));
    emrtpo_config = config_data["emrtpo_config"];
    start_frequency = config_data["start_frequency"];
    stop_frequency = config_data["stop_frequency"];
    n_frequencies = config_data["number_of_frequencies"];
    output_filename = config_data["output_filename"];
    assert_unambiguous_range = config_data["assert_unambiguous_range"];
    clean_temporary_files = config_data["clean_temporary_files"];

    for (size_t i = 0; i < config_data["attitude_origin"].size(); i++) {
      size_t id_row = i / 3;
      size_t id_col = i % 3;
      attitude_origin(id_row, id_col) = config_data["attitude_origin"][i];
    }

    for (size_t i = 0; i < config_data["attitude_target"].size(); i++) {
      size_t id_row = i / 3;
      size_t id_col = i % 3;
      attitude_origin(id_row, id_col) = config_data["attitude_origin"][i];
    }
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const VectorEMRTPOResultsConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "emrtpo_config": )" << config.emrtpo_config << ",\n";
  stream << R"(  "attitude_origin": )" << config.attitude_origin << ",\n";
  stream << R"(  "attitude_target": )" << config.attitude_origin << ",\n";
  stream << R"(  "start_frequency": )" << config.start_frequency << ",\n";
  stream << R"(  "stop_frequency": )" << config.stop_frequency << ",\n";
  stream << R"(  "n_frequencies": )" << config.n_frequencies << ",\n";
  stream << R"(  "output_filename": )" << config.output_filename << ",\n";
  stream << R"(  "clean_temporary_files": )" << config.clean_temporary_files << ",\n";

  stream << "}";
  return stream;
}

#endif//EMRTPO_PO_CONFIG_H
