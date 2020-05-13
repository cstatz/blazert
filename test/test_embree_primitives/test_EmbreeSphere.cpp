//
// Created by ogarten on 1/23/19.
//

#include "../../../rtcore/EmbreeDevice.h"
#include "../../../rtcore/EmbreeScene.h"
#include "../../../model/geometry/EmbreeSphere.h"
#include "../../catch.hpp"
#include "../../test_helpers.h"

using namespace em;
using namespace em::model::geometry;

TEST_CASE("Sphere", "[bounding box, distance to surface, intersections]")
{
  EmbreeDevice D("verbose=0");
  EmbreeScene S(D, RTC_SCENE_FLAG_NONE, 0);

  SECTION("BOUNDING BOX")
  {
    const Vec3f center{ 1.f, 1.f, 1.f };
    float radius = 1.f;

    EmbreeSphere sphere(D, S, center, radius);

    rtcCommitScene(S);

    RTCBounds bounds;
    rtcGetSceneBounds(S, &bounds);

    REQUIRE(bounds.lower_x == Approx(0.f));
    REQUIRE(bounds.lower_y == Approx(0.f));
    REQUIRE(bounds.lower_z == Approx(0.f));
    REQUIRE(bounds.upper_x == Approx(2.f));
    REQUIRE(bounds.upper_x == Approx(2.f));
    REQUIRE(bounds.upper_x == Approx(2.f));
  }

  SECTION("DISTANCE TO SURFACE")
  {
    Vec3f center{0.f, 0.f, 0.f};
    SECTION("R = 1")
    {
      float radius = 1.f;

      EmbreeSphere sphere(D, S, center, radius);
      rtcCommitScene(S);

      REQUIRE(sphere.distance_to_surface(Vec3f{ 0.f, 0.f, 0.f }) == Approx(1.f));
      REQUIRE(sphere.distance_to_surface(Vec3f{ -1.f, 0.f, 0.f }) == Approx(0.f));
      REQUIRE(sphere.distance_to_surface(Vec3f{ -2.f, 0.f, 0.f }) == Approx(1.f));
      REQUIRE(sphere.distance_to_surface(Vec3f{ -2.f, -2.f, -2.f }) == Approx(std::sqrt(3.f * 4.f) - 1.f));
    }
    SECTION("R = 3")
    {
      float radius = 3.f;

      //EmbreeDevice D("verbose=0");
      //EmbreeScene S(D, RTC_SCENE_FLAG_NONE, 0);
      EmbreeSphere sphere(D, S, center, radius);
      rtcCommitScene(S);

      REQUIRE(sphere.distance_to_surface(Vec3f{ 0.f, 0.f, 0.f }) == Approx(3.f));
      REQUIRE(sphere.distance_to_surface(Vec3f{ -1.f, 0.f, 0.f }) == Approx(2.f));
      REQUIRE(sphere.distance_to_surface(Vec3f{ -2.f, 0.f, 0.f }) == Approx(1.f));
      REQUIRE(sphere.distance_to_surface(Vec3f{ -2.f, -2.f, -2.f }) == Approx(std::sqrt(3.f * 4.f) - 3.f));
    }
  }
  SECTION("OCCLUDED")
  {
    Vec3f center{ 0.f, 0.f, 0.f };
    float radius = 1.f;

    EmbreeSphere sphere(D, S, center, radius);
    rtcCommitScene(S);

    Vec3f org1{ -2.f, 0.f, 0.f };
    Vec3f dir1{ 1.f, 0.f, 0.f };

    Vec3f org2{ 2.f, 2.f, 2.f };
    Vec3f dir2{ 1.f, 1.f, 1.f };

    Vec3f org3{ 2.f, 2.f, 2.f };
    Vec3f dir3{ -1.f, -1.f, -1.f };

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    RTCRay ray1{
      org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(), 0, 0, 0
    };
    RTCRay ray2{
      org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>().max(), 0, 0, 0
    };
    RTCRay ray3{
      org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<float>().max(), 0, 0, 0
    };

    // this ray hits, therefore tfar of the corresponding ray should be -inf
    rtcOccluded1(S, &context, &ray1);
    REQUIRE(std::isinf(-1 * ray1.tfar));

    // this ray does not hit, therefore the ray should be unchanged
    rtcOccluded1(S, &context, &ray2);
    REQUIRE(ray2.tfar == Approx(std::numeric_limits<float>().max()));

    // ray should hit
    rtcOccluded1(S, &context, &ray3);
    REQUIRE(std::isinf(-1 * ray3.tfar));
  }
  SECTION("INTERSECTIONS")
  {
    SECTION("Ray origin outside sphere")
    {
      Vec3f center1{ 0.f, 0.f, 0.f };
      Vec3f center2{ 0.f, 0.f, 5.f };
      float radius = 1.f;

      EmbreeSphere sphere1(D, S, center1, radius);
      EmbreeSphere sphere2(D, S, center2, radius);
      rtcCommitScene(S);

      Vec3f org1{ 2.f, 0.f, 0.f };
      Vec3f dir1{ -1.f, 0.f, 0.f };

      Vec3f org2{ 2.f, 0.f, 2.5f };
      Vec3f dir2{ -1.f, 0.f, 0.f };

      Vec3f org3{ 3.f, 0.f, 5.f };
      Vec3f dir3{ -1.f, 0.f, 0.f };

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{ org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ ray1, hit1 };
      rtcIntersect1(S, &context, &rayhit1);
      // should be in distance of 1
      REQUIRE(rayhit1.ray.tfar == Approx(1.f));

      RTCRay ray2{ org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ ray2, hit2 };
      rtcIntersect1(S, &context, &rayhit2);
      // should not hit, therefore tfar is the same as before
      REQUIRE(rayhit2.ray.tfar == Approx(std::numeric_limits<float>::max()));

      RTCRay ray3{ org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit3;
      hit3.geomID = RTC_INVALID_GEOMETRY_ID;
      hit3.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit3{ ray3, hit3 };
      rtcIntersect1(S, &context, &rayhit3);
      // should be in distance of 2
      REQUIRE(rayhit3.ray.tfar == Approx(2.f));
    }

    SECTION("Ray origin inside sphere")
    {
      Vec3f center1{0.f, 0.f, 0.f};
      float radius = 1.f;

      EmbreeSphere sphere1(D, S, center1, radius);
      rtcCommitScene(S);

      Vec3f org1{0.f, 0.f, 0.f};
      Vec3f dir1{-1.f, 0.f, 0.f};

      Vec3f org2{0.f, 0.f, 0.5f};
      Vec3f dir2{0.f, 0.f, -1.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{ org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ ray1, hit1 };
      rtcIntersect1(S, &context, &rayhit1);
      // should be in distance of 1
      REQUIRE(rayhit1.ray.tfar == Approx(1.f));

      RTCRay ray2{ org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ ray2, hit2 };
      rtcIntersect1(S, &context, &rayhit2);
      // should  hit in distance of 1.5
      REQUIRE(rayhit2.ray.tfar == Approx(1.5f));
    }
    SECTION("Ray origin on sphere")
    ///
    {
      Vec3f center1{1.f, 0.f, 0.f};
      float radius = 1.f;

      EmbreeSphere sphere1(D, S, center1, radius);
      rtcCommitScene(S);

      Vec3f org1{0.f, 0.f, 0.f};
      Vec3f dir1{1.f, 0.f, 0.f};

      Vec3f org2{0.f, 0.f, 0.f};
      Vec3f dir2{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{ org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ ray1, hit1 };
      rtcIntersect1(S, &context, &rayhit1);
      // should hit in distance of 2
      REQUIRE(rayhit1.ray.tfar == Approx(2.f));

      RTCRay ray2{ org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ ray2, hit2 };
      rtcIntersect1(S, &context, &rayhit2);
      // should not hit, ray points outward
      REQUIRE(rayhit2.ray.tfar == Approx(std::numeric_limits<float>::max()));
    }
    SECTION("ray passing through sphere center")
    {
      Vec3f center1{0.f, 0.f, 0.f};
      float radius = 2.f;

      EmbreeSphere sphere1(D, S, center1, radius);
      rtcCommitScene(S);

      Vec3f org1{3.f, 1.f, 0.f};
      Vec3f dir1{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{ org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                   0,       0,       0 };
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ ray1, hit1 };
      rtcIntersect1(S, &context, &rayhit1);

      REQUIRE(rayhit1.ray.tfar == Approx(1.26794919f));

      REQUIRE(rayhit1.hit.Ng_x == Approx(sqrt(3) / 2));
      REQUIRE(rayhit1.hit.Ng_y == Approx(sqrt(1) / 2));
      REQUIRE(rayhit1.hit.Ng_z == Approx(0.f));
    }
  }
}