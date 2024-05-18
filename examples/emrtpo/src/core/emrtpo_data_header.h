#pragma once
#ifndef EMRTPO_DATA_HEADER_H
#define EMRTPO_DATA_HEADER_H

#include <fstream>
#include <iostream>

using namespace blazert;

namespace emrt {

class VectorEMRTPOHeader {
public:
  uint8_t isize{8};
  uint8_t fsize{8};
  uint16_t major{2021};
  uint16_t minor{2};
  uint16_t revision{0};

  void read(std::ifstream &f) {
    uint64_t null{0};

    f.read(reinterpret_cast<char *>(&isize), sizeof(isize));
    f.read(reinterpret_cast<char *>(&fsize), sizeof(fsize));
    f.read(reinterpret_cast<char *>(&major), sizeof(major));
    f.read(reinterpret_cast<char *>(&minor), sizeof(minor));
    f.read(reinterpret_cast<char *>(&revision), sizeof(revision));
    f.read(reinterpret_cast<char *>(&null), sizeof(null));
    f.read(reinterpret_cast<char *>(&null), sizeof(null));
    f.read(reinterpret_cast<char *>(&null), sizeof(null));
    f.read(reinterpret_cast<char *>(&null), sizeof(null));
    f.read(reinterpret_cast<char *>(&null), sizeof(null));
    f.read(reinterpret_cast<char *>(&null), sizeof(null));
    f.read(reinterpret_cast<char *>(&null), sizeof(null));
  };

  void write(std::ofstream &f) const {
    uint64_t null{0};

    f.write(reinterpret_cast<const char *>(&isize), sizeof(isize));
    f.write(reinterpret_cast<const char *>(&fsize), sizeof(fsize));
    f.write(reinterpret_cast<const char *>(&major), sizeof(major));
    f.write(reinterpret_cast<const char *>(&minor), sizeof(minor));
    f.write(reinterpret_cast<const char *>(&revision), sizeof(revision));
    f.write(reinterpret_cast<const char *>(&null), sizeof(null));
    f.write(reinterpret_cast<const char *>(&null), sizeof(null));
    f.write(reinterpret_cast<const char *>(&null), sizeof(null));
    f.write(reinterpret_cast<const char *>(&null), sizeof(null));
    f.write(reinterpret_cast<const char *>(&null), sizeof(null));
    f.write(reinterpret_cast<const char *>(&null), sizeof(null));
    f.write(reinterpret_cast<const char *>(&null), sizeof(null));
  };
};

}// namespace emrt

#endif//EMRTPO_DATA_HEADER_H
