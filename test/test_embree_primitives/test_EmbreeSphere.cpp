//
// Created by ogarten on 1/23/19.
//
#include <third_party/doctest/doctest/doctest.h>
#include <blazert/embree/primitives/EmbreeSphere.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("EmbreeSphere", T, float) {
  auto device = rtcNewDevice("verbose=0,start_threads=1,threads=4,set_affinity=1");
  auto rtcscene = rtcNewScene(device);
  SUBCASE("BOUNDING BOX") {
    const Vec3r<T> center{1.f, 1.f, 1.f};
    T radius = 1.f;

    EmbreeSphere sphere(device, rtcscene, center, radius);

    rtcCommitScene(rtcscene);
    RTCBounds bounds;
    rtcGetSceneBounds(rtcscene, &bounds);

    CHECK(bounds.lower_x == Approx(0.f));
    CHECK(bounds.lower_y == Approx(0.f));
    CHECK(bounds.lower_z == Approx(0.f));
    CHECK(bounds.upper_x == Approx(2.f));
    CHECK(bounds.upper_x == Approx(2.f));
    CHECK(bounds.upper_x == Approx(2.f));
  }
  SUBCASE("DISTANCE TO SURFACE") {
    Vec3r<T> center{0.f, 0.f, 0.f};
    SUBCASE("R = 1") {
      T radius = 1.f;

      EmbreeSphere sphere(device, rtcscene, center, radius);

      rtcCommitScene(rtcscene);
      RTCBounds bounds;
      rtcGetSceneBounds(rtcscene, &bounds);

      CHECK(sphere.distance_to_surface(Vec3r<T>{0.f, 0.f, 0.f}) == Approx(1.f));
      CHECK(sphere.distance_to_surface(Vec3r<T>{-1.f, 0.f, 0.f}) == Approx(0.f));
      CHECK(sphere.distance_to_surface(Vec3r<T>{-2.f, 0.f, 0.f}) == Approx(1.f));
      CHECK(sphere.distance_to_surface(Vec3r<T>{-2.f, -2.f, -2.f}) == Approx(std::sqrt(3.f * 4.f) - 1.f));
    }
    SUBCASE("R = 3") {
      T radius = 3.f;
      EmbreeSphere sphere(device, rtcscene, center, radius);

      rtcCommitScene(rtcscene);
      RTCBounds bounds;
      rtcGetSceneBounds(rtcscene, &bounds);

      CHECK(sphere.distance_to_surface(Vec3r<T>{0.f, 0.f, 0.f}) == Approx(3.f));
      CHECK(sphere.distance_to_surface(Vec3r<T>{-1.f, 0.f, 0.f}) == Approx(2.f));
      CHECK(sphere.distance_to_surface(Vec3r<T>{-2.f, 0.f, 0.f}) == Approx(1.f));
      CHECK(sphere.distance_to_surface(Vec3r<T>{-2.f, -2.f, -2.f}) == Approx(std::sqrt(3.f * 4.f) - 3.f));
    }
  }

  SUBCASE("INTERSECTIONS") {
    SUBCASE("Ray origin outside sphere") {
      Vec3r<T> center1{0.f, 0.f, 0.f};
      Vec3r<T> center2{0.f, 0.f, 5.f};
      T radius = 1.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      EmbreeSphere sphere2(device, rtcscene, center2, radius);
      rtcCommitScene(rtcscene);

      Vec3r<T> org1{2.f, 0.f, 0.f};
      Vec3r<T> dir1{-1.f, 0.f, 0.f};

      Vec3r<T> org2{2.f, 0.f, 2.5f};
      Vec3r<T> dir2{-1.f, 0.f, 0.f};

      Vec3r<T> org3{3.f, 0.f, 5.f};
      Vec3r<T> dir3{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);
      // should be in distance of 1
      CHECK(rayhit1.ray.tfar == Approx(1.f));

      RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ray2, hit2};
      rtcIntersect1(rtcscene, &context, &rayhit2);
      // should not hit, therefore tfar is the same as before
      CHECK(rayhit2.ray.tfar == Approx(std::numeric_limits<T>::max()));

      RTCRay ray3{org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit3;
      hit3.geomID = RTC_INVALID_GEOMETRY_ID;
      hit3.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit3{ray3, hit3};
      rtcIntersect1(rtcscene, &context, &rayhit3);
      // should be in distance of 2
      CHECK(rayhit3.ray.tfar == Approx(2.f));
    }

    SUBCASE("Ray origin inside sphere") {
      Vec3r<T> center1{0.f, 0.f, 0.f};
      T radius = 1.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      rtcCommitScene(rtcscene);

      Vec3r<T> org1{0.f, 0.f, 0.f};
      Vec3r<T> dir1{-1.f, 0.f, 0.f};

      Vec3r<T> org2{0.f, 0.f, 0.5f};
      Vec3r<T> dir2{0.f, 0.f, -1.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);
      // should be in distance of 1
      CHECK(rayhit1.ray.tfar == Approx(1.f));

      RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ray2, hit2};
      rtcIntersect1(rtcscene, &context, &rayhit2);
      // should  hit in distance of 1.5
      CHECK(rayhit2.ray.tfar == Approx(1.5f));
    }
    SUBCASE("Ray origin on sphere")
    ///
    {
      Vec3r<T> center1{1.f, 0.f, 0.f};
      T radius = 1.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      rtcCommitScene(rtcscene);

      Vec3r<T> org1{0.f, 0.f, 0.f};
      Vec3r<T> dir1{1.f, 0.f, 0.f};

      Vec3r<T> org2{0.f, 0.f, 0.f};
      Vec3r<T> dir2{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);
      // should hit in distance of 2
      CHECK(rayhit1.ray.tfar == Approx(2.f));

      RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit2;
      hit2.geomID = RTC_INVALID_GEOMETRY_ID;
      hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit2{ray2, hit2};
      rtcIntersect1(rtcscene, &context, &rayhit2);
      // should not hit, ray points outward
      CHECK(rayhit2.ray.tfar == Approx(std::numeric_limits<T>::max()));
    }
    SUBCASE("ray passing through sphere center") {
      Vec3r<T> center1{0.f, 0.f, 0.f};
      T radius = 2.f;

      EmbreeSphere sphere1(device, rtcscene, center1, radius);
      rtcCommitScene(rtcscene);

      Vec3r<T> org1{3.f, 1.f, 0.f};
      Vec3r<T> dir1{-1.f, 0.f, 0.f};

      RTCIntersectContext context;
      rtcInitIntersectContext(&context);

      RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<T>().max(),
                  0, 0, 0};
      RTCHit hit1;
      hit1.geomID = RTC_INVALID_GEOMETRY_ID;
      hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
      RTCRayHit rayhit1{ray1, hit1};
      rtcIntersect1(rtcscene, &context, &rayhit1);

      CHECK(rayhit1.ray.tfar == Approx(1.26794919f));

      CHECK(rayhit1.hit.Ng_x == Approx(sqrt(3) / 2));
      CHECK(rayhit1.hit.Ng_y == Approx(sqrt(1) / 2));
      CHECK(rayhit1.hit.Ng_z == Approx(0.f));
    }
  }
}
