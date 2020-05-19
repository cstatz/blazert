//
// Created by orell on 1/12/18.
//

#ifndef BLAZERT_EMBREECYLINDER_H
#define BLAZERT_EMBREECYLINDER_H

#include <embree3/rtcore.h>
#include <embree3/rtcore_geometry.h>
#include <embree3/rtcore_scene.h>

#include <blazert/datatypes.h>

#include "EmbreeGeometryObject.h"

namespace blazert {

class EmbreeCylinder : public EmbreeGeometryObject
{
public:
  EmbreeCylinder(const RTCDevice& device,
                 const RTCScene& scene,
                 const Vec3r<float>& center,
                 const float a,
                 const float b,
                 const float height,
                 const Mat3r<float>& rotMatrix);
  ~EmbreeCylinder();

  const Vec3r<float>& center;
  const float a;
  const float b;
  const float height;
  const Mat3r<float>& rotMatrix;

  RTC_FORCEINLINE double distance_to_surface(
    const Vec3r<float>& point) const
  {
    /*
     * for arbitrary point:
     *  1. if point next to cylinder, distance to shell surface is distance
     * to circle on same z-axis
     *  2. if point above/below cylinder, distance to top/bottom one is
     * shortest
     *  3. if point is next to, and above/below, distance to top/bottom
     * circle perimeter is shortest
     */

    //        // point exactly on top surface
    //        if((point(2) == height) && (b*b*point(0)*point(0) +
    //        a*a*point(1)*point(1) <= a*a*b*b)) return 0;
    //        // point exactly on bottom surface
    //        if((point(2) == 0) && (b*b*point(0)*point(0) +
    //        a*a*point(1)*point(1) <= a*a*b*b)) return 0;
    //        // point on shell
    //        if((point(2) <= height) && (point(2) >= 0) &&
    //        (b*b*point(0)*point(0) + a*a*point(1)*point(1) == a*a*b*b))
    //        return 0;
    //
    //        // directly above
    //        if ((point(2) > height) && (b*b*point(0)*point(0) +
    //        a*a*point(1)*point(1) <= a*a*b*b))
    //            return abs(point(2) - height);
    //
    //        // directly below
    //        if ((point(2) < 0) && (b*b*point(0)*point(0) +
    //        a*a*point(1)*point(1) <= a*a*b*b))
    //            return abs(point(2));
    //
    //        // directly next to it
    //        if((point(2) <= height) && (point(2) >= 0) &&
    //        (b*b*point(0)*point(0) + a*a*point(1)*point(1) >= a*a*b*b))
    //            return 0;
    //
    //        // next to it but not in the same plane
    return 0;
  }
};

void
cylinderBoundsFunc(const RTCBoundsFunctionArguments* args);
void
cylinderIntersectFunc(const RTCIntersectFunctionNArguments* args);
void
cylinderOccludedFunc(const RTCOccludedFunctionNArguments* args);

}
#endif // BLAZERT_EMBREECYLINDER_H
