#pragma once
#ifndef EMRTPO_PATTERN_CONFIG_H
#define EMRTPO_PATTERN_CONFIG_H

#include <blazert/blazert.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "emrtpo_po_config.h"

using json = nlohmann::json;

using namespace std::complex_literals;

template<typename T>
class PatternConfig {
public:
  std::string transmitter_pattern_fname;
  std::string receiver_pattern_fname;

  explicit PatternConfig(const char *fname) {

    json config_data = json::parse(std::ifstream(fname));

    transmitter_pattern_fname = config_data['transmitter_pattern_file'];
    receiver_pattern_fname = config_data['receiver_pattern_file'];
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const PatternConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "transmitter_pattern_file": )" << config.transmitter_pattern_fname << ",\n";
  stream << R"(  "receiver_pattern_file": )" << config.receiver_pattern_fname << ",\n";

  stream << "}";
  return stream;
}

#endif//EMRTPO_PATTERN_CONFIG_H
