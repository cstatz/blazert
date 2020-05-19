//
// Created by Christoph Statz on 04.06.18.
//

#ifndef EM_ORIGINMESH_H
#define EM_ORIGINMESH_H

#include <blazert/datatypes.h>

class OriginMesh
{
public:
  OriginMesh() = default;
  virtual ~OriginMesh() = default;

  blazert::Vec3rList<float> vertices;
  blazert::Vec3iList triangles;

  virtual uint32_t triangle_count() const;

  virtual uint32_t vertices_count() const;

  void add_triangle(uint32_t a, uint32_t b, uint32_t c);

  void clear();

  void reserve(uint32_t t, uint32_t f);
};

#endif // EM_ORIGINMESH_H
