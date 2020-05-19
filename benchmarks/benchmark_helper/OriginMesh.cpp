//
// Created by Christoph Statz on 04.06.18.
//

#include "OriginMesh.h"


uint32_t
OriginMesh::triangle_count() const
{
  return triangles.size();
}

uint32_t
OriginMesh::vertices_count() const
{
  return vertices.size();
}

void
OriginMesh::add_triangle(uint32_t a, uint32_t b, uint32_t c)
{
  // First one is VISIT_CELL_TRI=1
  triangles.emplace_back(blazert::Vec3ui{a,b,c}); //
}

void
OriginMesh::clear()
{
  vertices.clear();
  triangles.clear();
}

void
OriginMesh::reserve(uint32_t t, uint32_t f)
{
  vertices.reserve(t);
  triangles.reserve(f);
}
