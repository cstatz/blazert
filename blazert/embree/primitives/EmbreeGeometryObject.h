//
// Created by Christoph Statz on 10.12.17.
//

#pragma once

#ifndef EM_EMBREEGEOMETRYOBJECT_H
#define EM_EMBREEGEOMETRYOBJECT_H

#include <blazert/datatypes.h>

namespace blazert {

class EmbreeGeometryObject {
protected:
  const RTCScene &scene;

public:
  EmbreeGeometryObject(const RTCScene &scene) : scene(scene){};

  RTCGeometry geometry;
  unsigned int geomID = static_cast<unsigned int>(-1);
  unsigned int kind = 0;
};

/***
 * This function sets the appropriate parameters for the ray hit structure.
 * @param rh        pointer RTCRayHit in which the arguments are saved
 * @param Ng        normalized normal vector
 * @param u
 * @param v
 * @param primID    primitiveID
 * @param geomID    geomID of the object
 * @param instID    instID of the object
 * @param tfar      length of the ray from source to intersection
 */

void inline setRayHit(RTCRayHit *rh, const Vec3r<float> &Ng, const float u, const float v, const unsigned int primID,
                      const unsigned int geomID, const unsigned int instID, const float tfar) {
  rh->hit.Ng_x = Ng[0];
  rh->hit.Ng_y = Ng[1];
  rh->hit.Ng_z = Ng[2];
  rh->hit.u = u;
  rh->hit.v = v;
  rh->hit.primID = primID;
  rh->hit.geomID = geomID;
  rh->hit.instID[0] = instID;
  rh->ray.tfar = tfar;//(t0 * dir).norm();
}

}// namespace blazert
#endif// EM_EMBREEGEOMETRYOBJECT_H