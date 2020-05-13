//
// Created by Christoph Statz on 17.01.17.
//
#pragma once

#ifndef EM_EMBREESPHERE_H
#define EM_EMBREESPHERE_H

#include <embree3/rtcore.h>
#include <embree3/rtcore_geometry.h>
#include <embree3/rtcore_scene.h>

#include "EmbreeGeometryObject.h"

namespace blazert {

class EmbreeSphere : public EmbreeGeometryObject
{
public:
  const Vec3f& center;
  const float radius;

public:
  EmbreeSphere(const EmbreeDevice& device,
               const EmbreeScene& scene,
               const Vec3f& center,
               float radius);

  ~EmbreeSphere() = default;

  inline double distance_to_surface(const Vec3f& point) const
  {
    const Vec3f& distance = center - point;
    return abs(norm(distance) - radius);
  }
};

void
sphereBoundsFunc(const RTCBoundsFunctionArguments* args);

void
sphereIntersectFunc(const RTCIntersectFunctionNArguments* args);

void
sphereOccludedFunc(const RTCOccludedFunctionNArguments* args);

} // namespace blazert

#endif // EM_EMBREESPHERE_H
