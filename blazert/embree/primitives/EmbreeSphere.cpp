//
// Created by Christoph Statz on 17.01.17.
//

#include "EmbreeSphere.h"
namespace blazert {

EmbreeSphere::EmbreeSphere(const RTCdevice& device,
                           const EmbreeScene& scene,
                           const Vec3f& center,
                           float radius)
  : EmbreeGeometryObject(scene)
  , center(center)
  , radius(radius)
{
  // create geomentry of user defined type
  geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

  rtcSetGeometryUserPrimitiveCount(geometry, 1); // one primitive
  rtcSetGeometryUserData(geometry, this);

  // set bounds, intersect and occluded functions for user geometry
  rtcSetGeometryBoundsFunction(geometry, sphereBoundsFunc, this);
  rtcSetGeometryIntersectFunction(geometry, sphereIntersectFunc);
  rtcSetGeometryOccludedFunction(geometry, sphereOccludedFunc);

  // commit changes on geometry
  rtcCommitGeometry(geometry);
  // attach geomentry to scene and get geomID
  geomID = rtcAttachGeometry(scene, geometry);
}

/***
 * Calculate boundary box around the sphere. This is needed for building the
 * acceleration structure
 *
 * @param args BoundsFunctionArguments can be found in embree library, function
 * will not be called directly
 */
void
sphereBoundsFunc(const RTCBoundsFunctionArguments* args)
{
  const EmbreeSphere& sphere =
    ((EmbreeSphere*)(args->geometryUserPtr))[args->primID];

  const Vec3f center = sphere.center;

  args->bounds_o->lower_x = center[0] - (sphere.radius);
  args->bounds_o->lower_y = center[1] - (sphere.radius);
  args->bounds_o->lower_z = center[2] - (sphere.radius);
  args->bounds_o->upper_x = center[0] + (sphere.radius);
  args->bounds_o->upper_y = center[1] + (sphere.radius);
  args->bounds_o->upper_z = center[2] + (sphere.radius);
}

/***
 * This function calculates the intersection point between a ray and the sphere.
 * The results are saved in the RTCRayHit structure which is provided by the
 * RTCIntsersectFunctionNArguments
 *
 * @param args
 */
void
sphereIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  // if N != 1, this function is not applicable (this only runs in debug mode)
  assert(args->N == 1);

  // load correct sphere from data ptr
  const EmbreeSphere& sphere =
    ((const EmbreeSphere*)(args->geometryUserPtr))[args->primID];
  const Vec3f center = sphere.center;

  const unsigned int primID = args->primID;
  const unsigned int instID = args->context->instID[0];
  const unsigned int geomID = sphere.geomID;

  // return if the ray is declared as not valid
  if (!(args->valid[0]))
    return;

  // loads the ray and hit structure for the following calculation
  // see occludedFunc to do this differently
  RTCRayHit* rayhit = (RTCRayHit*)args->rayhit;
  RTCRay* ray = &(rayhit->ray);

  const Vec3f org{ray->org_x, ray->org_y, ray->org_z};
  const Vec3f dir{ray->dir_x, ray->dir_y, ray->dir_z};
  const Vec3f v = org - center;

  // calculate the actual intersections
  const float A = dot(dir, dir);
  const float B = 2.0f * dot(v, dir); //
  const float C =
    dot(v, v) - sphere.radius * sphere.radius; // distance to sphere
  const float D = B * B - 4.0f * A * C;

  // interpretation of D: if D < 0 the ray misses the sphere and therefore
  // does not intersect term in the square root becomes negative
  if (D < 0.0f)
    return;

  const float Q = sqrtf(D);
  const float rcpA = 1.f / A;
  const float t0 = 0.5f * rcpA * (-B - Q);
  const float t1 = 0.5f * rcpA * (-B + Q);

  // t0 corresponds to the first hit
  if ((ray->tnear < t0) && (t0 < ray->tfar)) {
    const Vec3f Ng = normalize(org + t0 * dir - center);

    setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
  }
  // t1 corresponds to the first hit
  if ((ray->tnear < t1) && (t1 < ray->tfar)) {
    const Vec3f Ng = normalize(org + t1 * dir - center);

    setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
  }
}

/***
 * This function tests whether the ray hits the sphere or not. However, it does
 * not provide any information about the intersection.
 *
 * @param args
 */
void
sphereOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  // if N != 1, this function is not applicable
  assert(args->N == 1);

  // load correct sphere from data ptr
  const EmbreeSphere& sphere =
    ((EmbreeSphere*)(args->geometryUserPtr))[args->primID];
  const Vec3f center = sphere.center;

  // return if the ray is declared as not valid
  if (!(args->valid[0]))
    return;

  // load corresponding ray data, N = 1 (only one ray)
  const float org_x = RTCRayN_org_x(args->ray, 1, 0);
  const float org_y = RTCRayN_org_y(args->ray, 1, 0);
  const float org_z = RTCRayN_org_z(args->ray, 1, 0);
  const Vec3f org{org_x, org_y, org_z};

  const float dir_x = RTCRayN_dir_x(args->ray, 1, 0);
  const float dir_y = RTCRayN_dir_y(args->ray, 1, 0);
  const float dir_z = RTCRayN_dir_z(args->ray, 1, 0);
  const Vec3f dir{dir_x, dir_y, dir_z};

  const Vec3f v = org - center;

  const float A = dot(dir, dir);
  const float B = 2.0f * dot(v, dir);
  const float C = dot(v, v) - sphere.radius * sphere.radius;
  const float D = B * B - 4.0f * A * C;

  if (D < 0.0f)
    return;

  const float Q = sqrtf(D);
  const float rcpA = 1.f / A;
  const float t0 = 0.5f * rcpA * (-B - Q);
  const float t1 = 0.5f * rcpA * (-B + Q);

  // if any hit occurred, tfar of the ray should be set to -inf
  if ((RTCRayN_tnear(args->ray, 1, 0) < t0) &&
      (t0 < RTCRayN_tfar(args->ray, 1, 0))) {
    RTCRayN_tfar(args->ray,
                 args->N,
                 0) = -std::numeric_limits<float>()
                         .infinity(); // std::numeric_limits<float>().max();
  }
  if ((RTCRayN_tnear(args->ray, 1, 0) < t0) &&
      (t1 < RTCRayN_tfar(args->ray, 1, 0))) {
    RTCRayN_tfar(args->ray, args->N, 0) =
      -std::numeric_limits<float>().infinity();
  }
}

} // namespace blazert