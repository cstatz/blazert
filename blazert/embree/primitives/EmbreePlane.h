//
// Created by ogarten on 2/11/19.
//

#ifndef BLAZERT_EMBREEPLANE_H
#define BLAZERT_EMBREEPLANE_H

#include <embree3/rtcore.h>
#include <embree3/rtcore_geometry.h>
#include <embree3/rtcore_scene.h>

#include <cmath>

#include "EmbreeGeometryObject.h"

namespace blazert {

class EmbreePlane : public EmbreeGeometryObject {
public:
  const Vec3r<float> &planeCenter;
  const float d1;
  const float d2;
  const Mat3r<float> &rot;

  const float thickness = std::numeric_limits<float>::min();

public:
  EmbreePlane(const RTCDevice &device,
              const RTCScene &scene,
              const Vec3r<float> &planeCenter,
              const float d1,
              const float d2,
              const Mat3r<float> &rot);

  ~EmbreePlane() = default;

  inline double distance_to_surface(const Vec3r<float> &point) const { return 0.0; }
};

void planeBoundsFunc(const RTCBoundsFunctionArguments *args);

void planeIntersectFunc(const RTCIntersectFunctionNArguments *args);

void planeOccludedFunc(const RTCOccludedFunctionNArguments *args);

}// namespace blazert

#endif// BLAZERT_EMBREEPLANE_H
