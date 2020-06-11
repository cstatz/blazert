//
// Created by orell on 1/12/18.
//

#include "EmbreeCylinder.h"

#include <algorithm>
#include <blazert/datatypes.h>

namespace blazert {

/***
 * This constructs a cylinder where the center point is on the bottom ellipse.
 * @param scene EmbreeScene in which the object exists
 * @param center vector to the middle point of the ellipse
 * @param a semi axis in the direction of x
 * @param b semi axis in the direction of y
 * @param height height of the cylinder
 * @param rotMatrix rotation matrix to get arbitrary object placement
 */
EmbreeCylinder::EmbreeCylinder(const RTCDevice& device,
                               const RTCScene& scene,
                               const Vec3r<float>& center,
                               const float a,
                               const float b,
                               const float height,
                               const Mat3r<float>& rotMatrix)
  : EmbreeGeometryObject(scene)
  , center(center)
  , a(a)
  , b(b)
  , height(height)
  , rotMatrix(rotMatrix)
{
  // create geomentry of user defined type
  geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

  rtcSetGeometryUserPrimitiveCount(geometry, 1); // one primitive
  rtcSetGeometryUserData(geometry, this);

  // set bounds, intersect and occluded functions for user geometry
  rtcSetGeometryBoundsFunction(geometry, cylinderBoundsFunc, this);
  rtcSetGeometryIntersectFunction(geometry, cylinderIntersectFunc);
  rtcSetGeometryOccludedFunction(geometry, cylinderOccludedFunc);

  // commit changes on geometry
  rtcCommitGeometry(geometry);
  // attach geomentry to scene and get geomID
  geomID = rtcAttachGeometry(scene, geometry);
}

EmbreeCylinder::~EmbreeCylinder() = default;

void
cylinderBoundsFunc(const RTCBoundsFunctionArguments* args)
{
  const EmbreeCylinder& cylinder = ((EmbreeCylinder*)(args->geometryUserPtr))[args->primID];
  const Mat3r<float> &rot = cylinder.rotMatrix;

  // rot.transposeInPlace();

  /* Proposed Algorithm to calculate rotated bounding box
   * 1. construct necessary vectors which describe the shape in the coordinate
   * system local to the object
   * 2. transform all of these vectors with the given rotation matrix
   * 3. find the maximum dimension in each direction
   *
   * For a three dimensional object, three vectors are necessary to describe
   * the object in its local coordinate system a : half axis in direction of x
   * from center to perimeter b : half axis in direction of y from center to
   * perimeter h : vector from center to top surface center : points to the
   * center of the center of the cylinder
   */

  const Vec3r<float> &center = cylinder.center;
  const Vec3r<float> &a1_tmp{cylinder.a, 0, 0};
  const Vec3r<float> &b1_tmp{0, cylinder.b, 0};
  const Vec3r<float> &h1_tmp{0, 0, cylinder.height/2};
//  const Vec3r<float> &a2_tmp{cylinder.a, 0, 0};
//  const Vec3r<float> &b2_tmp{0, cylinder.b, 0};
//  const Vec3r<float> &h2_tmp{0, 0, cylinder.height/2};

  // These vectors describe the cylinder in the global coordinate system
  const Vec3r<float> &a1 = center + rot * a1_tmp;
  const Vec3r<float> &b1 = center + rot * b1_tmp;
  const Vec3r<float> &h1 = center + rot * h1_tmp;

  const Vec3r<float> &a2 = center - rot * a1_tmp;
  const Vec3r<float> &b2 = center - rot * b1_tmp;
  const Vec3r<float> &h2 = center - rot * h1_tmp;

  // maximum / minimum is also the max/min of the bounding box
  args->bounds_o->lower_x = std::min({ a1[0], b1[0], a2[0], b2[0], h1[0], h2[0] });
  args->bounds_o->lower_y = std::min({ a1[1], b1[1], a2[1], b2[1], h1[1], h2[1] });
  args->bounds_o->lower_z = std::min({ a1[2], b1[2], a2[2], b2[2], h1[2], h2[2] });
  args->bounds_o->upper_x = std::max({ a1[0], b1[0], a2[0], b2[0], h1[0], h2[0] });
  args->bounds_o->upper_y = std::max({ a1[1], b1[1], a2[1], b2[1], h1[1], h2[1] });
  args->bounds_o->upper_z = std::max({ a1[2], b1[2], a2[2], b2[2], h1[2], h2[2] });
}

void
cylinderIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  const EmbreeCylinder& cylinder = ((EmbreeCylinder*)(args->geometryUserPtr))[args->primID];

  // get rotation matrix and transpose it
  const Mat3r<float> &rot = cylinder.rotMatrix;
  const Mat3r<float> &inverse_rot = trans(rot);

  const unsigned int primID = args->primID;
  const unsigned int instID = args->context->instID[0];
  const unsigned int geomID = cylinder.geomID;

  // return if the ray is declared as not valid
  if (!(args->valid[0]))
    return;

  // loads the ray and hit structure for the following calculation
  // see occludedFunc to do this differently
  RTCRayHit* rayhit = (RTCRayHit*)args->rayhit;
  RTCRay* ray = &(rayhit->ray);

  const Vec3r<float> org_tmp{ray->org_x, ray->org_y, ray->org_z};
  const Vec3r<float> dir_tmp{ray->dir_x, ray->dir_y, ray->dir_z};

  // center on coordinate system of object
  // also copy origin and
  const Vec3r<float> &org = inverse_rot * (org_tmp - cylinder.center);
  const Vec3r<float> &dir = inverse_rot * dir_tmp;

  // cylinder does not have to have a circle as base area, can also be ellipse
  // test in which area the source is
  /*
   *                 1
   *               -----..........
   *              |     |
   *              |  4  |
   *              |     |   3
   *              |     |
   *               -----..........
   *                 2
   *
   *
   */

  const float x0 = org[0];
  const float y0 = org[1];
  const float z0 = org[2];

  const float j = dir[0];
  const float k = dir[1];
  const float l = dir[2];

  const float a = cylinder.a;
  const float b = cylinder.b;
  const float h = cylinder.height;

  // area 1
  if (z0 > h/2.) {
    if (l == 0)
      return;
    const float t0 = (h/2 - z0) / l;
    if (t0 < 0)
      return;
    const Vec3r<float> &intercept = org + t0 * dir;

    // intercept point in circle
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      const Vec3r<float> normal{0.f, 0.f, 1.f};
      const Vec3r<float> Ng = rot * normal / norm(normal);

      setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));

    } else {
      const float A = a * a * k * k + b * b * j * j;
      const float B = a * a * k * y0 + b * b * j * x0;
      const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return;
      const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const float t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<float> &intercept0 = org + dir * t0;
      const Vec3r<float> &intercept1 = org + dir * t1;

      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h/2) && (intercept1[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h/2) && (intercept0[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
      }
    }
  } else
    // area 2
    if (z0 < -h/2) {
    if (l == 0)
      return;
    const float t0 = (-h/2 - z0) / l;
    if (t0 < 0)
      return;

    const Vec3r<float> &intercept = org + t0 * dir;

    // intercept point in circle
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      const Vec3r<float> &normal{0.f, 0.f, -1.f};
      const Vec3r<float> &Ng = rot * normal / norm(normal);

      setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
    } else {
      const float A = a * a * k * k + b * b * j * j;
      const float B = a * a * k * y0 + b * b * j * x0;
      const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return;
      const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const float t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<float> &intercept0 = org + dir * t0;
      const Vec3r<float> &intercept1 = org + dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h/2) && (intercept1[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h/2) && (intercept0[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
      }
    }
  }
  // area 3
  else if ((z0 > -h/2) && (z0 < h/2) && (x0 * x0 * b * b + y0 * y0 * a * a > a * a * b * b)) {
    const float A = a * a * k * k + b * b * j * j;
    const float B = a * a * k * y0 + b * b * j * x0;
    const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

    if (C < 0)
      return;
    const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
    const float t1 = -1.0 / A * (B + a * b * sqrt(C));

    const Vec3r<float> &intercept0 = org + dir * t0;
    const Vec3r<float> &intercept1 = org + dir * t1;
    // t1 is right intercept
    if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h/2) && (intercept1[2] >= -h/2)) {
      const Vec3r<float> &normal{2* intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
      const Vec3r<float> &Ng = rot * normal / norm(normal);

      setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
    }
    if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h/2) && (intercept0[2] >= -h/20)) {
      const Vec3r<float> &normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
      const Vec3r<float> &Ng = rot * normal / norm(normal);

      setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
    }
  }
  // area 4
  if ((z0 >= -h/2) && (z0 <= h/2) && (x0 * x0 * b * b + y0 * y0 * a * a <= a * a * b * b)) {
    // only surrounding surface can have an intersection
    if (l == 0) {
      // if the z-component of the direction vector points along the
      // bottom or top circle, we will not have a intersection
      if ((z0 == -h/2) || (z0 == h/2))
        return;

      const float A = a * a * k * k + b * b * j * j;
      const float B = a * a * k * y0 + b * b * j * x0;
      const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return;
      const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const float t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<float> &intercept0 = org + dir * t0;
      const Vec3r<float> &intercept1 = org + dir * t1;
      // t1 is right intercept

      if ((t1 > 0) && (intercept1[2] <= h/2) && (intercept1[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
      }
      if ((t0 > 0) && (intercept0[2] <= h/2) && (intercept0[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
      }
    }
    // only happens if l != 0, an k, j == 0
    // only bottom or top can have intersection
    else if ((j == 0.f) && (k == 0.f)) {
      // if ray points along the circular surface, intresesction not
      // possible
      if (b * b * x0 * x0 + a * a * y0 * y0 == a * a * b * b)
        return;
      // bottom circle
      if (l < 0) {
        const float t0 = (-h/2 - z0) / l;

        const Vec3r<float> &normal{0.f, 0.f, -1.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
      }
      // top circle
      if (l > 0) {
        const float t0 = (h/2 - z0) / l;

        const Vec3r<float> &normal{0.f, 0.f, 1.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
      }
    }
    // all cases where j, k and l are unequal to zero
    // all surfaces can have an intersection
    else {
      const float A = a * a * k * k + b * b * j * j;
      const float B = a * a * k * y0 + b * b * j * x0;
      const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return;
      const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const float t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<float> &intercept0 = org + dir * t0;
      const Vec3r<float> &intercept1 = org + dir * t1;
      // t1 is right intercept

      if ((t1 > 0) && (intercept1[2] <= h/2) && (intercept1[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t1 * dir));
        return;
      }
      if ((t0 > 0) && (intercept0[2] <= h/2) && (intercept0[2] >= -h/2)) {
        const Vec3r<float> &normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
        return;
      }

      // bottom circle
      if (l < 0) {
        const float t0 = (-h/2 - z0) / l;

        const Vec3r<float> &normal{0.f, 0.f, -1.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
      }
      // top circle
      if (l > 0) {
        const float t0 = (h/2 - z0) / l;

        const Vec3r<float> &normal{0.f, 0.f, 1.f};
        const Vec3r<float> &Ng = rot * normal / norm(normal);

        setRayHit(rayhit, Ng, 0.f, 0.f, primID, geomID, instID, norm(t0 * dir));
      }
    }
  }
}

void
cylinderOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  const EmbreeCylinder& cylinder = ((EmbreeCylinder*)(args->geometryUserPtr))[args->primID];

  // get rotation matrix and transpose it
  const Mat3r<float> &rot = cylinder.rotMatrix;
  const Mat3r<float> &inverse_rot = trans(rot);

  // return if the ray is declared as not valid
  if (!(args->valid[0]))
    return;

  // loads the ray and hit structure for the following calculation
  // see occludedFunc to do this differently

  RTCRay* ray = (RTCRay*)(args->ray);
  const Vec3r<float> &org_tmp{ray->org_x, ray->org_y, ray->org_z};
  const Vec3r<float> &dir_tmp{ray->dir_x, ray->dir_y, ray->dir_z};

  // center on coordinate system of object
  // also copy origin and
  const Vec3r<float> &org = inverse_rot * (org_tmp - cylinder.center);
  const Vec3r<float> &dir = inverse_rot * dir_tmp;
  //const Vec3r<float> center = rot * cylinder.center;

  // cylinder does not have to have a circle as base area, can also be ellipse
  // test in which area the source is
  /*
   *                 1
   *               -----..........
   *              |     |
   *              |  4  |
   *              |     |   3
   *              |     |
   *               -----..........
   *                 2
   *
   * note: the case that the source is inside the cylinder is not taken into
   * account yet
   */

  const float x0 = org[0];
  const float y0 = org[1];
  const float z0 = org[2];

  const float j = dir[0];
  const float k = dir[1];
  const float l = dir[2];

  const float a = cylinder.a;
  const float b = cylinder.b;
  const float h = cylinder.height;

  // area 1
  // if the ray does not hit the they cylinder if it does not hit the plane
  // which bounds the cylinder on the top.
  if (z0 > h/2) {
    if (l == 0)
      return; // ray parallel to the circular surface
    const float t0 = (h/2 - z0) / l;
    if (t0 < 0)
      return; // no intersection
    const Vec3r<float> &intercept = org + t0 * dir;

    // intersection in top ellipse
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      ray->tfar = -std::numeric_limits<float>().infinity();
      return;
    } else {
      const float A = a * a * k * k + b * b * j * j;
      const float B = a * a * k * y0 + b * b * j * x0;
      const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return;
      const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const float t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<float> &intercept0 = org + dir * t0;
      const Vec3r<float> &intercept1 = org + dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h/2) && (intercept1[2] >= -h/2)) {
        ray->tfar = -std::numeric_limits<float>().infinity();
        return;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h/2) && (intercept0[2] >= -h/2)) {
        ray->tfar = -std::numeric_limits<float>().infinity();
        return;
      }
    }
  } else // area 2
    if (z0 < -h/2) {
    if (l == 0)
      return;
    const float t0 = (-h/2 - z0) / l;
    if (t0 < 0)
      return;
    const Vec3r<float> &intercept = org + t0 * dir;

    // intersection in top ellipse
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      ray->tfar = -std::numeric_limits<float>().infinity();
      return;
    } else {
      const float A = a * a * k * k + b * b * j * j;
      const float B = a * a * k * y0 + b * b * j * x0;
      const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return;

      const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const float t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<float> &intercept0 = org + dir * t0;
      const Vec3r<float> &intercept1 = org + dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h/2) && (intercept1[2] >= -h/2)) {
        ray->tfar = -std::numeric_limits<float>().infinity();
        return;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h/2) && (intercept0[2] >= -h/2)) {
        ray->tfar = -std::numeric_limits<float>().infinity();
        return;
      }
    }

  } else // area 3
    if ((z0 > -h/2) && (z0 < h/2) && (x0 * x0 * b * b + y0 * y0 * a * a > a * a * b * b)) {
    const float A = a * a * k * k + b * b * j * j;
    const float B = a * a * k * y0 + b * b * j * x0;
    const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

    if (C < 0)
      return;
    const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
    const float t1 = -1.0 / A * (B + a * b * sqrt(C));

    if ((t0 < 0) && (t1 < 0))
      return; // no intersection if both t's change the direction of the
              // ray
    // intersection
    ray->tfar = -std::numeric_limits<float>().infinity();
  }
  if ((z0 >= -h/2) && (z0 <= h/2) && (x0 * x0 * b * b + y0 * y0 * a * a <= a * a * b * b)) {
    // if ray origin is on bottom or top circle and points outwards, there
    // is no intersection
    if (((z0 == -h/2) && (l < 0)) || ((z0 == h/2) && (l > 0)))
      return;

    // if ray origin is on shell surface, t0 and t1 need to be GREATER than
    // zero, otherwise there is no intersection
    const float A = a * a * k * k + b * b * j * j;
    const float B = a * a * k * y0 + b * b * j * x0;
    const float C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

    if (C < 0)
      return;
    const float t0 = 1.0 / A * (-B + a * b * sqrt(C));
    const float t1 = -1.0 / A * (B + a * b * sqrt(C));

    if ((t0 <= 0) && (t1 <= 0))
      return; // no intersection if both t's change the direction of the
              // ray

    // if no return criteria is met, there will be a intresection
    ray->tfar = -std::numeric_limits<float>().infinity();
  }
}

}