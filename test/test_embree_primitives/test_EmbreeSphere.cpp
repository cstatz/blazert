//
// Created by ogarten on 1/23/19.
//

#include "../catch.hpp"
#include <blazert/embree/primitives/EmbreeSphere.h>

using namespace blazert;

TEMPLATE_TEST_CASE("EmbreeSphere", "[bounding box, distance to surface, intersections]", float) {
  auto device = rtcNewDevice("verbose=0,start_threads=1,threads=4,set_affinity=1");
  auto rtcscene = rtcNewScene(device);
  SECTION("BOUNDING BOX") {
    const Vec3r<float> center{1.f, 1.f, 1.f};
    float radius = 1.f;

    EmbreeSphere sphere(device, rtcscene, center, radius);

    rtcCommitScene(rtcscene);
    RTCBounds bounds;
    rtcGetSceneBounds(rtcscene, &bounds);

    REQUIRE(bounds.lower_x == Approx(0.f));
    REQUIRE(bounds.lower_y == Approx(0.f));
    REQUIRE(bounds.lower_z == Approx(0.f));
    REQUIRE(bounds.upper_x == Approx(2.f));
    REQUIRE(bounds.upper_x == Approx(2.f));
    REQUIRE(bounds.upper_x == Approx(2.f));
  }
  SECTION("DISTANCE TO SURFACE") {
    Vec3r<float> center{0.f, 0.f, 0.f};
    SECTION("R = 1") {
      float radius = 1.f;

      EmbreeSphere sphere(device, rtcscene, center, radius);

      rtcCommitScene(rtcscene);
      RTCBounds bounds;
      rtcGetSceneBounds(rtcscene, &bounds);

      REQUIRE(sphere.distance_to_surface(Vec3r<float>{0.f, 0.f, 0.f}) == Approx(1.f));
      REQUIRE(sphere.distance_to_surface(Vec3r<float>{-1.f, 0.f, 0.f}) == Approx(0.f));
      REQUIRE(sphere.distance_to_surface(Vec3r<float>{-2.f, 0.f, 0.f}) == Approx(1.f));
      REQUIRE(sphere.distance_to_surface(Vec3r<float>{-2.f, -2.f, -2.f}) == Approx(std::sqrt(3.f * 4.f) - 1.f));
    }
    SECTION("R = 3") {
      float radius = 3.f;
      EmbreeSphere sphere(device, rtcscene, center, radius);

      rtcCommitScene(rtcscene);
      RTCBounds bounds;
      rtcGetSceneBounds(rtcscene, &bounds);

      REQUIRE(sphere.distance_to_surface(Vec3r<float>{0.f, 0.f, 0.f}) == Approx(3.f));
      REQUIRE(sphere.distance_to_surface(Vec3r<float>{-1.f, 0.f, 0.f}) == Approx(2.f));
      REQUIRE(sphere.distance_to_surface(Vec3r<float>{-2.f, 0.f, 0.f}) == Approx(1.f));
      REQUIRE(sphere.distance_to_surface(Vec3r<float>{-2.f, -2.f, -2.f}) == Approx(std::sqrt(3.f * 4.f) - 3.f));
    }
  }

  SECTION("INTERSECTIONS") {
    SECTION("Ray origin outside sphere") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      Vec3r<TestType> center2{0.f, 0.f, 5.f};
      float radius = 1.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      EmbreeSphere sphere2(device, rtcscene, center2, radius);
      rtcCommitScene(rtcscene);

      Vec3r<TestType> org1{2.f, 0.f, 0.f};
      Vec3r<TestType> dir1{-1.f, 0.f, 0.f};

      Vec3r<TestType> org2{2.f, 0.f, 2.5f};
      Vec3r<TestType> dir2{-1.f, 0.f, 0.f};

      Vec3r<TestType> org3{3.f, 0.f, 5.f};
      Vec3r<TestType> dir3{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);
      // should be in distance of 1
      REQUIRE(rayhit1.ray.tfar == Approx(1.f));

      RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ray2, hit2};
      rtcIntersect1(rtcscene, &context, &rayhit2);
      // should not hit, therefore tfar is the same as before
      REQUIRE(rayhit2.ray.tfar == Approx(std::numeric_limits<float>::max()));

      RTCRay ray3{org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit3;
      hit3.geomID = RTC_INVALID_GEOMETRY_ID;
      hit3.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit3{ray3, hit3};
      rtcIntersect1(rtcscene, &context, &rayhit3);
      // should be in distance of 2
      REQUIRE(rayhit3.ray.tfar == Approx(2.f));
    }

    SECTION("Ray origin inside sphere") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      float radius = 1.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      rtcCommitScene(rtcscene);

      Vec3r<TestType> org1{0.f, 0.f, 0.f};
      Vec3r<TestType> dir1{-1.f, 0.f, 0.f};

      Vec3r<TestType> org2{0.f, 0.f, 0.5f};
      Vec3r<TestType> dir2{0.f, 0.f, -1.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);
      // should be in distance of 1
      REQUIRE(rayhit1.ray.tfar == Approx(1.f));

      RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ray2, hit2};
      rtcIntersect1(rtcscene, &context, &rayhit2);
      // should  hit in distance of 1.5
      REQUIRE(rayhit2.ray.tfar == Approx(1.5f));
    }
    SECTION("Ray origin on sphere")
    ///
    {
      Vec3r<TestType> center1{1.f, 0.f, 0.f};
      float radius = 1.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      rtcCommitScene(rtcscene);

      Vec3r<TestType> org1{0.f, 0.f, 0.f};
      Vec3r<TestType> dir1{1.f, 0.f, 0.f};

      Vec3r<TestType> org2{0.f, 0.f, 0.f};
      Vec3r<TestType> dir2{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);
      // should hit in distance of 2
      REQUIRE(rayhit1.ray.tfar == Approx(2.f));

      RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ray2, hit2};
      rtcIntersect1(rtcscene, &context, &rayhit2);
      // should not hit, ray points outward
      REQUIRE(rayhit2.ray.tfar == Approx(std::numeric_limits<float>::max()));
    }
    SECTION("ray passing through sphere center") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      float radius = 2.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      rtcCommitScene(rtcscene);

      Vec3r<TestType> org1{3.f, 1.f, 0.f};
      Vec3r<TestType> dir1{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);

      REQUIRE(rayhit1.ray.tfar == Approx(1.26794919f));

      REQUIRE(rayhit1.hit.Ng_x == Approx(sqrt(3) / 2));
      REQUIRE(rayhit1.hit.Ng_y == Approx(sqrt(1) / 2));
      REQUIRE(rayhit1.hit.Ng_z == Approx(0.f));
    }
  }
}