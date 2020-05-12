//
// Copyright 2012-2015, Syoyo Fujita.
//
// Licensed under 2-clause BSD liecense.
//
#ifndef _TINY_OBJ_LOADER_H
#define _TINY_OBJ_LOADER_H

#include <string>
#include <vector>
#include <map>

namespace tinyobj {

typedef struct {
  std::vector<double> positions;
  std::vector<double> normals;
  std::vector<double> texcoords;
  std::vector<unsigned int> indices;
  std::vector<int> material_ids; // per-mesh material ID
} mesh_t;

typedef struct {
  std::string name;
  mesh_t mesh;
} shape_t;

std::string LoadObj(std::vector<shape_t> &shapes,       // [output]
                    const char *filename);

}
#endif // _TINY_OBJ_LOADER_H
