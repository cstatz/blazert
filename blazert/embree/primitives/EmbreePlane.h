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

inline void planeBoundsFunc(const RTCBoundsFunctionArguments *args);
inline void planeIntersectFunc(const RTCIntersectFunctionNArguments *args);
inline void planeOccludedFunc(const RTCOccludedFunctionNArguments *args);

class EmbreePlane : public EmbreeGeometryObject {
public:
  const Vec3r<float> &planeCenter;
  const float d1;
  const float d2;
  const Mat3r<float> &rot;

  const float thickness = std::numeric_limits<float>::min();

public:
  EmbreePlane(const RTCDevice &device, const RTCScene &scene, const Vec3r<float> &planeCenter, const float d1,
              const float d2, const Mat3r<float> &rot) noexcept;
  ~EmbreePlane() = default;

  inline double distance_to_surface(const Vec3r<float> &point) const { return 0.0; }
};

/***
* This constructs an EmbreePlane object. By default the plane is parallel to
* the xy-plane. It dimension in x-direction is d1 while the dimension in
* y-direction is d2. The unit vector in x, y and z direction serve as the base
* vectors and the normal vector of the plane respectivly. The position is
* specified by the vector planeCenter which points to the center of the plane
* and by the rotation matrix rot which rotates the plane. Rotation is applied
* before the translation to the center
*
* Although this puts the burden on the user of the class, this ensures easy
* intersection testing which leads to higher performance.
*
* @param device        EmbreeDevice on which the geometry lives
* @param scene         EmbreeScene which the geometry will be attached to
* @param planeCenter   center of the plane
* @param d1            dimension in x-direction
* @param d2            dimension in y-direction
* @param rot           rotation matrix
*/
inline EmbreePlane::EmbreePlane(const RTCDevice &device, const RTCScene &scene, const Vec3r<float> &planeCenter,
                                const float d1, const float d2, const Mat3r<float> &rot) noexcept
    : EmbreeGeometryObject(scene), planeCenter(planeCenter), d1(d1), d2(d2), rot(rot) {
  // create geometry of user defined type
  geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

  rtcSetGeometryUserPrimitiveCount(geometry, 1);// one primitive
  rtcSetGeometryUserData(geometry, this);

  // set bounds, intersect and occluded functions for user geometry
  rtcSetGeometryBoundsFunction(geometry, planeBoundsFunc, this);
  rtcSetGeometryIntersectFunction(geometry, planeIntersectFunc);
  rtcSetGeometryOccludedFunction(geometry, planeOccludedFunc);

  // commit changes on geometry
  rtcCommitGeometry(geometry);
  // attach geometry to scene and get geomID
  geomID = rtcAttachGeometry(scene, geometry);
}

/***
 * Calculate boundary box around the plane. This is needed for building the
 * acceleration structure
 *
 * @param args BoundsFunctionArguments can be found in embree library, function
 * will not be called directly
 */
inline void planeBoundsFunc(const RTCBoundsFunctionArguments *args) {
  const EmbreePlane &plane = ((EmbreePlane *) (args->geometryUserPtr))[args->primID];

  Mat3r<float> rot = plane.rot;

  const Vec3r<float> planeCenter = plane.planeCenter;

  // vectors describing extent in x direction
  const Vec3r<float> &a1_tmp{-plane.d1 / 2.f, -plane.d2 / 2.f, 0.f};
  const Vec3r<float> &a2_tmp{plane.d1 / 2.f, -plane.d2 / 2.f, 0.f};
  // vectors describing extent in y direction
  const Vec3r<float> &a3_tmp{-plane.d1 / 2.f, plane.d2 / 2.f, 0.f};
  const Vec3r<float> &a4_tmp{plane.d1 / 2.f, plane.d2 / 2.f, 0.f};

  // vectors describing extent in y direction
  const Vec3r<float> &c1_tmp{0.f, 0.f, -plane.thickness};
  const Vec3r<float> &c2_tmp{0.f, 0.f, plane.thickness};

  // std::cout << rot_internal << std::endl;
  // vectors describing the plane in the global coordinate system
  const Vec3r<float> &a1 = planeCenter + rot * a1_tmp;
  const Vec3r<float> &a2 = planeCenter + rot * a2_tmp;
  const Vec3r<float> &a3 = planeCenter + rot * a3_tmp;
  const Vec3r<float> &a4 = planeCenter + rot * a4_tmp;
  const Vec3r<float> &c1 = planeCenter + rot * c1_tmp;
  const Vec3r<float> &c2 = planeCenter + rot * c2_tmp;

  // maximum / minimum is also the max/min of the bounding box
  args->bounds_o->lower_x = std::min({a1[0], a2[0], a3[0], a4[0], c1[0], c2[0]});
  args->bounds_o->lower_y = std::min({a1[1], a2[1], a3[1], a4[1], c1[1], c2[1]});
  args->bounds_o->lower_z = std::min({a1[2], a2[2], a3[2], a4[2], c1[2], c2[2]});
  args->bounds_o->upper_x = std::max({a1[0], a2[0], a3[0], a4[0], c1[0], c2[0]});
  args->bounds_o->upper_y = std::max({a1[1], a2[1], a3[1], a4[1], c1[1], c2[1]});
  args->bounds_o->upper_z = std::max({a1[2], a2[2], a3[2], a4[2], c1[2], c2[2]});
}

/***
 * This function calculates the intersection point between a ray and the plane.
 * The results are saved in the RTCRayHit structure which is provided by the
 * RTCIntsersectFunctionNArguments
 *
 * @param args
 */
inline void planeIntersectFunc(const RTCIntersectFunctionNArguments *args) {
  // load correct plane from data ptr
  const EmbreePlane &plane = ((const EmbreePlane *) (args->geometryUserPtr))[args->primID];

  const Mat3r<float> &rot = plane.rot;
  const Mat3r<float> &inverse_rot = trans(rot);

  const unsigned int primID = args->primID;
  const unsigned int instID = args->context->instID[0];
  const unsigned int geomID = plane.geomID;

  // return if the ray is declared as not valid
  if (!(args->valid[0]))
    return;

  // loads the ray and hit structure for the following calculation
  // see occludedFunc to do this differently
  RTCRayHit *rayhit = (RTCRayHit *) args->rayhit;
  RTCRay *ray = &(rayhit->ray);

  // transform to local coordinate system
  const Vec3r<float> &org_tmp{ray->org_x, ray->org_y, ray->org_z};
  const Vec3r<float> &dir_tmp{ray->dir_x, ray->dir_y, ray->dir_z};

  const Vec3r<float> &org = inverse_rot * (org_tmp - plane.planeCenter);
  const Vec3r<float> &dir = inverse_rot * dir_tmp;

  // calculate interception point
  const float t1 = -org[2] / dir[2];
  const Vec3r<float> &intercept = org + t1 * dir;

  const float tnear = rayhit->ray.tnear;
  const float tfar = rayhit->ray.tfar;

  const float x_min = -plane.d1 / 2;
  const float x_max = plane.d1 / 2;

  const float y_min = -plane.d2 / 2;
  const float y_max = plane.d2 / 2;

  // is intercept within ray limits?
  // does it actually hit the plane?
  if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
      && (intercept[1] < y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{0.0, 0.0, 1.f};
    setRayHit(rayhit, org[2] / abs(org[2]) * Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  }
  // plane edges
  else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
           && (intercept[1] < y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{-1.f, 0.0, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] > y_min)
             && (intercept[1] < y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{1.f, 0.0, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] == y_min)
             && (intercept[1] < y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{0.f, -1.f, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
             && (intercept[1] == y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{0.f, 1.f, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  }

  // plane corners
  else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] == y_min)
           && (intercept[1] < y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{-1.f, -1.f, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
             && (intercept[1] == y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{-1.f, 1.f, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] == y_min)
             && (intercept[1] < y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{1.f, -1.f, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] > y_min)
             && (intercept[1] == y_max)) {
    const Vec3r<float> &Ng = rot * Vec3r<float>{1.f, 1.f, org[2] / abs(org[2])};
    setRayHit(rayhit, Ng / norm(Ng), 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  }
}

/***
 * This function tests whether the ray hits the plane or not. However, it does
 * not provide any information about the intersection.
 *
 * @param args
 */
inline void planeOccludedFunc(const RTCOccludedFunctionNArguments *args) {
  // if N != 1, this function is not applicable
  assert(args->N == 1);

  // load correct plane from data ptr
  const EmbreePlane &plane = ((const EmbreePlane *) (args->geometryUserPtr))[args->primID];

  Mat3r<float> rot = plane.rot;
  transpose(rot);

  // return if the ray is declared as not valid
  if (!(args->valid[0]))
    return;

  // loads the ray and hit structure for the following calculation
  // see occludedFunc to do this differently
  RTCRay *ray = (RTCRay *) (args->ray);

  // transform to local coordinate system
  const Vec3r<float> org_tmp{ray->org_x, ray->org_y, ray->org_z};
  const Vec3r<float> dir_tmp{ray->dir_x, ray->dir_y, ray->dir_z};

  const Vec3r<float> org = rot * (org_tmp - plane.planeCenter);
  const Vec3r<float> dir = rot * dir_tmp;

  // calculate interception point
  const float t1 = -org[2] / dir[2];
  const Vec3r<float> intercept = org + t1 * dir;

  const float tnear = ray->tnear;
  const float tfar = ray->tfar;

  const float x_min = -plane.d1 / 2;
  const float x_max = plane.d1 / 2;

  const float y_min = -plane.d2 / 2;
  const float y_max = plane.d2 / 2;

  // is intercept within ray limits?
  // does it actually hit the plane?
  if ((t1 > tnear) && (t1 < tfar) && (intercept[0] >= x_min) && (intercept[0] <= x_max) && (intercept[1] >= y_min)
      && (intercept[1] <= y_max)) {
    ray->tfar = -std::numeric_limits<float>().infinity();
  }
}

}// namespace blazert

#endif// BLAZERT_EMBREEPLANE_H
