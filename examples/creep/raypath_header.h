#pragma once
#ifndef EM_TRACER_RAYPATH_HEADER_H
#define EM_TRACER_RAYPATH_HEADER_H

#include <fstream>
#include <iostream>

using namespace blazert;

namespace emrt {

class RayPathHeader {
public:
  uint8_t isize{8};
  uint8_t fsize{8};
  uint16_t major{2021};
  uint16_t minor{2};
  uint16_t revision{8};

  void read(std::ifstream &f) {
    uint64_t null{0};

    f.read((char *) (&isize), sizeof(isize));
    f.read((char *) (&fsize), sizeof(fsize));
    f.read((char *) (&major), sizeof(major));
    f.read((char *) (&minor), sizeof(minor));
    f.read((char *) (&revision), sizeof(revision));
    f.read((char *) (&null), sizeof(null));
    f.read((char *) (&null), sizeof(null));
    f.read((char *) (&null), sizeof(null));
    f.read((char *) (&null), sizeof(null));
    f.read((char *) (&null), sizeof(null));
    f.read((char *) (&null), sizeof(null));
    f.read((char *) (&null), sizeof(null));
  };

  void write(std::ofstream &f) const {
    uint64_t null{0};

    f.write((char *) (&isize), sizeof(isize));
    f.write((char *) (&fsize), sizeof(fsize));
    f.write((char *) (&major), sizeof(major));
    f.write((char *) (&minor), sizeof(minor));
    f.write((char *) (&revision), sizeof(revision));
    f.write((char *) (&null), sizeof(null));
    f.write((char *) (&null), sizeof(null));
    f.write((char *) (&null), sizeof(null));
    f.write((char *) (&null), sizeof(null));
    f.write((char *) (&null), sizeof(null));
    f.write((char *) (&null), sizeof(null));
    f.write((char *) (&null), sizeof(null));
  };
};

}// namespace emrt

#endif//EM_TRACER_RAYPATH_HEADER_H
