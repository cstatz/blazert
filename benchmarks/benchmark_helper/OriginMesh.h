//
// Created by Christoph Statz on 04.06.18.
//

#ifndef EM_ORIGINMESH_H
#define EM_ORIGINMESH_H

#include <blazert/datatypes.h>

template<typename T>
class OriginMesh
{
public:
  OriginMesh() = default;
  virtual ~OriginMesh() = default;

  blazert::Vec3rList<T> vertices;
  blazert::Vec3iList triangles;

  virtual uint32_t triangle_count() const;

  virtual uint32_t vertices_count() const;

  void add_triangle(uint32_t a, uint32_t b, uint32_t c);

  void clear();

  void reserve(uint32_t t, uint32_t f);
};

template<typename T>
uint32_t OriginMesh<T>::triangle_count() const
{
  return triangles.size();
}

template<typename T>
uint32_t OriginMesh<T>::vertices_count() const
{
  return vertices.size();
}

template<typename T>
void OriginMesh<T>::add_triangle(uint32_t a, uint32_t b, uint32_t c)
{
  // First one is VISIT_CELL_TRI=1
  triangles.emplace_back(blazert::Vec3ui{a,b,c}); //
}

template<typename T>
void OriginMesh<T>::clear()
{
  vertices.clear();
  triangles.clear();
}

template<typename T>
void OriginMesh<T>::reserve(uint32_t t, uint32_t f)
{
  vertices.reserve(t);
  triangles.reserve(f);
}

#endif // EM_ORIGINMESH_H
