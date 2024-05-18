#pragma once
#ifndef EMRTPO_FLD_CONFIG_H
#define EMRTPO_FLD_CONFIG_H

#include <blazert/blazert.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using json = nlohmann::json;

using namespace std::complex_literals;

template<typename T>
class FLDConfig {
public:
  double length;

  explicit FLDConfig(const char *fname) {

    json config_data = json::parse(std::ifstream(fname));
    length = config_data['length'];
  }
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const FLDConfig<T> &config) {
  stream << "{\n";

  stream << R"(  "length": )" << config.length << ",\n";

  stream << "}";
  return stream;
}

#endif//EMRTPO_FLD_CONFIG_H
