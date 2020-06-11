//
// Created by ogarten on 14/05/2020.
//

#include <blazert/embree/primitives/EmbreePlane.h>
#include <third_party/doctest/doctest/doctest.h>
#include "../test_helpers.h"

using namespace blazert;
using namespace doctest;


TEST_CASE_TEMPLATE("EmbreePlane", T, float) {

  const T d1 = 2.f;
  const T d2 = 2.f;

  auto D = rtcNewDevice("verbose=0,start_threads=1,threads=4,set_affinity=1");
  auto S = rtcNewScene(D);

  SUBCASE("bounding box") {
    SUBCASE("center at origin") {
      Vec3r<T> center{0.f, 0.f, 0.f};
      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        // plane z = 0, -2<x,y<2
        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-1.f));
        CHECK(bound.lower_y == Approx(-1.f));
        CHECK(bound.lower_z == Approx(-std::numeric_limits<float>::min()));
        CHECK(bound.upper_x == Approx(1.f));
        CHECK(bound.upper_y == Approx(1.f));
        CHECK(bound.upper_z == Approx(std::numeric_limits<float>::min()));
      }
      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};

        // defines plane x=0
        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-std::numeric_limits<float>::min()));
        CHECK(bound.lower_y == Approx(-1.f));
        CHECK(bound.lower_z == Approx(-1.f));
        CHECK(bound.upper_x == Approx(std::numeric_limits<float>::min()));
        CHECK(bound.upper_y == Approx(1.f));
        CHECK(bound.upper_z == Approx(1.f));
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {
        // plane on z=0, later rotated to x = 0
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<T> axis{static_cast<T>(1 /  std::sqrt(2)), static_cast<T>(1 /  std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-1.f));
        CHECK(bound.lower_y == Approx(-1.f));
        CHECK(bound.lower_z == Approx(-std::sqrt(2)));
        CHECK(bound.upper_x == Approx(1.f));
        CHECK(bound.upper_y == Approx(1.f));
        CHECK(bound.upper_z == Approx(std::sqrt(2)));
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<T> axis{0, 0, 1};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 4);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-std::sqrt(2)));
        CHECK(bound.lower_y == Approx(-std::sqrt(2)));
        CHECK(bound.lower_z == Approx(0));
        CHECK(bound.upper_x == Approx(std::sqrt(2)));
        CHECK(bound.upper_y == Approx(std::sqrt(2)));
        CHECK(bound.upper_z == Approx(0));
      }
    }
    SUBCASE("shifted center") {
      Vec3r<T> center{1.f, 1.f, 1.f};
      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        // plane z = 0, -2<x,y<2
        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(0.f));
        CHECK(bound.lower_y == Approx(0.f));
        CHECK(bound.lower_z == Approx(1.f));
        CHECK(bound.upper_x == Approx(2.f));
        CHECK(bound.upper_y == Approx(2.f));
        CHECK(bound.upper_z == Approx(1.f));
      }
      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};

        // defines plane x=0
        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(1.f));
        CHECK(bound.lower_y == Approx(0.f));
        CHECK(bound.lower_z == Approx(0.f));
        CHECK(bound.upper_x == Approx(1.f));
        CHECK(bound.upper_y == Approx(2.f));
        CHECK(bound.upper_z == Approx(2.f));
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {
        // plane on z=0, later rotated to x = 0
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<T> axis{static_cast<T>(1 /  std::sqrt(2)), static_cast<T>(1 /  std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(0));
        CHECK(bound.lower_y == Approx(0));
        CHECK(bound.lower_z == Approx(1 -  std::sqrt(2)));
        CHECK(bound.upper_x == Approx(2));
        CHECK(bound.upper_y == Approx(2));
        CHECK(bound.upper_z == Approx(1 +  std::sqrt(2)));
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<T> axis{0, 0, 1};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 4);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(1 - std::sqrt(2)));
        CHECK(bound.lower_y == Approx(1 - std::sqrt(2)));
        CHECK(bound.lower_z == Approx(1));
        CHECK(bound.upper_x == Approx(1 + std::sqrt(2)));
        CHECK(bound.upper_y == Approx(1 + std::sqrt(2)));
        CHECK(bound.upper_z == Approx(1));
      }
    }
  }
  SUBCASE("INTERSECTS") {
    SUBCASE("center at origin") {
      Vec3r<T> center{0.f, 0.f, 0.f};
      SUBCASE("non-rotated"){
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        Vec3r<T> org1{0.f, 0.f, 5.f};
        Vec3r<T> dir1{0.f, 0.f, -1.f};
        RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
            0, 0, 0};
        RTCHit hit1;
        hit1.geomID = RTC_INVALID_GEOMETRY_ID;
        hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
        RTCRayHit rayhit1{ray1, hit1};
        rtcIntersect1(S, &context, &rayhit1);

        CHECK(rayhit1.ray.tfar == Approx(5));
        CHECK(rayhit1.hit.Ng_x == Approx(0.f));
        CHECK(rayhit1.hit.Ng_y == Approx(0.f));
        CHECK(rayhit1.hit.Ng_z == Approx(1.f));
      }
      SUBCASE("rotated about (0,1,0)") {
        // matrix which rotates the plane 45 degrees about the z-axis ( x = 0 is now
        // plane eq)
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        Vec3r<T> org1{5.f, 0.f, 0.f};
        Vec3r<T> dir1{-1.f, 0.f, 0.f};
        RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
            0, 0, 0};
        RTCHit hit1;
        hit1.geomID = RTC_INVALID_GEOMETRY_ID;
        hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
        RTCRayHit rayhit1{ray1, hit1};
        rtcIntersect1(S, &context, &rayhit1);

        CHECK(rayhit1.ray.tfar == Approx(5));
        CHECK(rayhit1.hit.Ng_x == Approx(1.f));
        CHECK(rayhit1.hit.Ng_y == Approx(0.f));
        CHECK(rayhit1.hit.Ng_z == Approx(0.f));
      }
      SUBCASE("rotated about normalized(1,1,0), edge hit") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<T> axis{static_cast<T>(1 / std::sqrt(2)), static_cast<T>(1 / std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        Vec3r<T> org1{-5.f, 5.f, 0.5f};
        Vec3r<T> dir1{1.f, -1.f, 0.f};
        RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
            0, 0, 0};
        RTCHit hit1;
        hit1.geomID = RTC_INVALID_GEOMETRY_ID;
        hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
        RTCRayHit rayhit1{ray1, hit1};
        rtcIntersect1(S, &context, &rayhit1);

        // hits from the negative x direction -> normal vector should point towards
        // that direction
        CHECK(rayhit1.ray.tfar == Approx(5 * std::sqrt(2)));
        CHECK(rayhit1.hit.Ng_x == Approx(-1 / std::sqrt(2)));
        CHECK(rayhit1.hit.Ng_y == Approx(1 / std::sqrt(2)));
        CHECK(rayhit1.hit.Ng_z == Approx(0.f));// for rotated planes, you can expect small numerical
        // error of size 1e16
      }
      SUBCASE("non-rotated, ray origin outside plane boundaries") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        SUBCASE("outside on positive x-axis") {
          Vec3r<T> org1{4.f, 0.f, 4.f};
          Vec3r<T> dir1{-1.f, 0.f, -1.f};
          RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit1;
          hit1.geomID = RTC_INVALID_GEOMETRY_ID;
          hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit1{ray1, hit1};
          rtcIntersect1(S, &context, &rayhit1);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit1.ray.tfar == Approx(4 * std::sqrt(2.f)));
          CHECK(rayhit1.hit.Ng_x == Approx(0.f));
          CHECK(rayhit1.hit.Ng_y == Approx(0.f));
          CHECK(rayhit1.hit.Ng_z == Approx(1.f));
        }
        SUBCASE("outside on positive x-axis") {
          Vec3r<T> org2{-4.f, 0.f, -4.f};
          Vec3r<T> dir2{1.f, 0.f, 1.f};
          RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit2;
          hit2.geomID = RTC_INVALID_GEOMETRY_ID;
          hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit2{ray2, hit2};
          rtcIntersect1(S, &context, &rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit2.ray.tfar == Approx(4. * std::sqrt(2.f)));

          CHECK(rayhit2.hit.Ng_x == Approx(0.f));
          CHECK(rayhit2.hit.Ng_y == Approx(0.f));
          CHECK(rayhit2.hit.Ng_z == Approx(-1.f));
        }
      }
      SUBCASE("non-rotated, edge intersection") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);
        SUBCASE("edge: x max") {
          // edge at x_max
          Vec3r<T> org1{1.f, 0.f, 4.f};
          Vec3r<T> dir1{0.f, 0.f, -1.f};
          RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit1;
          hit1.geomID = RTC_INVALID_GEOMETRY_ID;
          hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit1{ray1, hit1};
          rtcIntersect1(S, &context, &rayhit1);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit1.ray.tfar == Approx(4));
          CHECK(rayhit1.hit.Ng_x == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit1.hit.Ng_y == Approx(0.f));
          CHECK(rayhit1.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: x min") {
          // edge at x_min
          Vec3r<T> org2{-1.f, 0.f, 4.f};
          Vec3r<T> dir2{0.f, 0.f, -1.f};
          RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit2;
          hit2.geomID = RTC_INVALID_GEOMETRY_ID;
          hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit2{ray2, hit2};

          rtcIntersect1(S, &context, &rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit2.ray.tfar == Approx(4));
          CHECK(rayhit2.hit.Ng_x == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit2.hit.Ng_y == Approx(0.f));
          CHECK(rayhit2.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y max") {
          // edge at y_max
          Vec3r<T> org3{0.f, 1.f, 4.f};
          Vec3r<T> dir3{0.f, 0.f, -1.f};
          RTCRay ray3{org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit3;
          hit3.geomID = RTC_INVALID_GEOMETRY_ID;
          hit3.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit3{ray3, hit3};
          rtcIntersect1(S, &context, &rayhit3);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit3.ray.tfar == Approx(4));
          CHECK(rayhit3.hit.Ng_x == Approx(0.f));
          CHECK(rayhit3.hit.Ng_y == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit3.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y min") {      // edge at y_min
          Vec3r<T> org4{0.f, -1.f, 4.f};
          Vec3r<T> dir4{0.f, 0.f, -1.f};
          RTCRay ray4{org4[0], org4[1], org4[2], 0, dir4[0], dir4[1], dir4[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit4;
          hit4.geomID = RTC_INVALID_GEOMETRY_ID;
          hit4.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit4{ray4, hit4};
          rtcIntersect1(S, &context, &rayhit4);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit4.ray.tfar == Approx(4));
          CHECK(rayhit4.hit.Ng_x == Approx(0.f));
          CHECK(rayhit4.hit.Ng_y == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit4.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
      }
      SUBCASE("non-rotated, corner intersection") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);
        SUBCASE("corner: x max, y max") {
          // corner at x_max, y_max
          Vec3r<T> org1{1.f, 1.f, 4.f};
          Vec3r<T> dir1{0.f, 0.f, -1.f};
          RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit1;
          hit1.geomID = RTC_INVALID_GEOMETRY_ID;
          hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit1{ray1, hit1};
          rtcIntersect1(S, &context, &rayhit1);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit1.ray.tfar == Approx(4));
          CHECK(rayhit1.hit.Ng_x == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit1.hit.Ng_y == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit1.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y max") {
          // corner at x_min, y_max
          Vec3r<T> org2{-1.f, 1.f, 4.f};
          Vec3r<T> dir2{0.f, 0.f, -1.f};
          RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit2;
          hit2.geomID = RTC_INVALID_GEOMETRY_ID;
          hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit2{ray2, hit2};
          rtcIntersect1(S, &context, &rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit2.ray.tfar == Approx(4));

          CHECK(rayhit2.hit.Ng_x == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit2.hit.Ng_y == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit2.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x max, y min") {
          // corner at x_max, y_min
          Vec3r<T> org3{1.f, -1.f, 4.f};
          Vec3r<T> dir3{0.f, 0.f, -1.f};
          RTCRay ray3{org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit3;
          hit3.geomID = RTC_INVALID_GEOMETRY_ID;
          hit3.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit3{ray3, hit3};
          rtcIntersect1(S, &context, &rayhit3);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit3.ray.tfar == Approx(4));
          CHECK(rayhit3.hit.Ng_x == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit3.hit.Ng_y == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit3.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y min") {
          // corner at x_min, y_min
          Vec3r<T> org4{-1.f, -1.f, 4.f};
          Vec3r<T> dir4{0.f, 0.f, -1.f};
          RTCRay ray4{org4[0], org4[1], org4[2], 0, dir4[0], dir4[1], dir4[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit4;
          hit4.geomID = RTC_INVALID_GEOMETRY_ID;
          hit4.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit4{ray4, hit4};
          rtcIntersect1(S, &context, &rayhit4);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit4.ray.tfar == Approx(4));
          CHECK(rayhit4.hit.Ng_x == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit4.hit.Ng_y == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit4.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
      }
    }
    SUBCASE("shifted center") {
      Vec3r<T> center{1.f, 1.f, 1.f};
      SUBCASE("non-rotated"){
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        Vec3r<T> org1{1.f, 1.f, 5.f};
        Vec3r<T> dir1{0.f, 0.f, -1.f};
        RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
            0, 0, 0};
        RTCHit hit1;
        hit1.geomID = RTC_INVALID_GEOMETRY_ID;
        hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
        RTCRayHit rayhit1{ray1, hit1};
        rtcIntersect1(S, &context, &rayhit1);

        CHECK(rayhit1.ray.tfar == Approx(4));
        CHECK(rayhit1.hit.Ng_x == Approx(0.f));
        CHECK(rayhit1.hit.Ng_y == Approx(0.f));
        CHECK(rayhit1.hit.Ng_z == Approx(1.f));
      }
      SUBCASE("rotated about (0,1,0)") {
        // matrix which rotates the plane 45 degrees about the z-axis ( x = 0 is now
        // plane eq)
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        Vec3r<T> org1{5.f, 1.f, 1.f};
        Vec3r<T> dir1{-1.f, 0.f, 0.f};
        RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
            0, 0, 0};
        RTCHit hit1;
        hit1.geomID = RTC_INVALID_GEOMETRY_ID;
        hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
        RTCRayHit rayhit1{ray1, hit1};
        rtcIntersect1(S, &context, &rayhit1);

        CHECK(rayhit1.ray.tfar == Approx(4));
        CHECK(rayhit1.hit.Ng_x == Approx(1.f));
        CHECK(rayhit1.hit.Ng_y == Approx(0.f));
        CHECK(rayhit1.hit.Ng_z == Approx(0.f));
      }
      SUBCASE("rotated about normalized(1,1,0), edge hit") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<T> axis{static_cast<T>(1 / std::sqrt(2)), static_cast<T>(1 / std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        Vec3r<T> org1{-4.f, 6.f, 0.5f};
        Vec3r<T> dir1{1.f, -1.f, 0.f};
        RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
            0, 0, 0};
        RTCHit hit1;
        hit1.geomID = RTC_INVALID_GEOMETRY_ID;
        hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
        RTCRayHit rayhit1{ray1, hit1};
        rtcIntersect1(S, &context, &rayhit1);

        // hits from the negative x direction -> normal vector should point towards
        // that direction
        CHECK(rayhit1.ray.tfar == Approx(5 * std::sqrt(2)));
        CHECK(rayhit1.hit.Ng_x == Approx(-1 / std::sqrt(2)));
        CHECK(rayhit1.hit.Ng_y == Approx(1 / std::sqrt(2)));
        CHECK(rayhit1.hit.Ng_z == Approx(0.f));// for rotated planes, you can expect small numerical
        // error of size 1e16
      }
      SUBCASE("non-rotated, ray origin outside plane boundaries") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        SUBCASE("outside on positive x-axis") {
          Vec3r<T> org1{5.f, 1.f, 5.f};
          Vec3r<T> dir1{-1.f, 0.f, -1.f};
          RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit1;
          hit1.geomID = RTC_INVALID_GEOMETRY_ID;
          hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit1{ray1, hit1};
          rtcIntersect1(S, &context, &rayhit1);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit1.ray.tfar == Approx(4 * std::sqrt(2.f)));
          CHECK(rayhit1.hit.Ng_x == Approx(0.f));
          CHECK(rayhit1.hit.Ng_y == Approx(0.f));
          CHECK(rayhit1.hit.Ng_z == Approx(1.f));
        }
        SUBCASE("outside on negative x-axis") {
          Vec3r<T> org2{-3.f, 1.f, -3.f};
          Vec3r<T> dir2{1.f, 0.f, 1.f};
          RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit2;
          hit2.geomID = RTC_INVALID_GEOMETRY_ID;
          hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit2{ray2, hit2};
          rtcIntersect1(S, &context, &rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit2.ray.tfar == Approx(4. * std::sqrt(2.f)));

          CHECK(rayhit2.hit.Ng_x == Approx(0.f));
          CHECK(rayhit2.hit.Ng_y == Approx(0.f));
          CHECK(rayhit2.hit.Ng_z == Approx(-1.f));
        }
      }
      SUBCASE("non-rotated, edge intersection") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);
        SUBCASE("edge: x max") {
          // edge at x_max
          Vec3r<T> org1{2.f, 1.f, 4.f};
          Vec3r<T> dir1{0.f, 0.f, -1.f};
          RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit1;
          hit1.geomID = RTC_INVALID_GEOMETRY_ID;
          hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit1{ray1, hit1};
          rtcIntersect1(S, &context, &rayhit1);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit1.ray.tfar == Approx(3));
          CHECK(rayhit1.hit.Ng_x == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit1.hit.Ng_y == Approx(0.f));
          CHECK(rayhit1.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: x min") {
          // edge at x_min
          Vec3r<T> org2{0.f, 1.f, 4.f};
          Vec3r<T> dir2{0.f, 0.f, -1.f};
          RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit2;
          hit2.geomID = RTC_INVALID_GEOMETRY_ID;
          hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit2{ray2, hit2};

          rtcIntersect1(S, &context, &rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit2.ray.tfar == Approx(3));
          CHECK(rayhit2.hit.Ng_x == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit2.hit.Ng_y == Approx(0.f));
          CHECK(rayhit2.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y max") {
          // edge at y_max
          Vec3r<T> org3{1.f, 2.f, 4.f};
          Vec3r<T> dir3{0.f, 0.f, -1.f};
          RTCRay ray3{org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit3;
          hit3.geomID = RTC_INVALID_GEOMETRY_ID;
          hit3.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit3{ray3, hit3};
          rtcIntersect1(S, &context, &rayhit3);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit3.ray.tfar == Approx(3));
          CHECK(rayhit3.hit.Ng_x == Approx(0.f));
          CHECK(rayhit3.hit.Ng_y == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit3.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y min") {      // edge at y_min
          Vec3r<T> org4{1.f, 0.f, 4.f};
          Vec3r<T> dir4{0.f, 0.f, -1.f};
          RTCRay ray4{org4[0], org4[1], org4[2], 0, dir4[0], dir4[1], dir4[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit4;
          hit4.geomID = RTC_INVALID_GEOMETRY_ID;
          hit4.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit4{ray4, hit4};
          rtcIntersect1(S, &context, &rayhit4);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit4.ray.tfar == Approx(3));
          CHECK(rayhit4.hit.Ng_x == Approx(0.f));
          CHECK(rayhit4.hit.Ng_y == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit4.hit.Ng_z == Approx(1.f / std::sqrt(2)));
        }
      }
      SUBCASE("non-rotated, corner intersection") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);

        EmbreePlane plane(D, S, center, d1, d2, rot);
        rtcCommitScene(S);

        RTCIntersectContext context;
        rtcInitIntersectContext(&context);
        SUBCASE("corner: x max, y max") {
          // corner at x_max, y_max
          Vec3r<T> org1{2.f, 2.f, 4.f};
          Vec3r<T> dir1{0.f, 0.f, -1.f};
          RTCRay ray1{org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit1;
          hit1.geomID = RTC_INVALID_GEOMETRY_ID;
          hit1.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit1{ray1, hit1};
          rtcIntersect1(S, &context, &rayhit1);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit1.ray.tfar == Approx(3));
          CHECK(rayhit1.hit.Ng_x == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit1.hit.Ng_y == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit1.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y max") {
          // corner at x_min, y_max
          Vec3r<T> org2{0.f, 2.f, 4.f};
          Vec3r<T> dir2{0.f, 0.f, -1.f};
          RTCRay ray2{org2[0], org2[1], org2[2], 0, dir2[0], dir2[1], dir2[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit2;
          hit2.geomID = RTC_INVALID_GEOMETRY_ID;
          hit2.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit2{ray2, hit2};
          rtcIntersect1(S, &context, &rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit2.ray.tfar == Approx(3));

          CHECK(rayhit2.hit.Ng_x == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit2.hit.Ng_y == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit2.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x max, y min") {
          // corner at x_max, y_min
          Vec3r<T> org3{2.f, 0.f, 4.f};
          Vec3r<T> dir3{0.f, 0.f, -1.f};
          RTCRay ray3{org3[0], org3[1], org3[2], 0, dir3[0], dir3[1], dir3[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit3;
          hit3.geomID = RTC_INVALID_GEOMETRY_ID;
          hit3.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit3{ray3, hit3};
          rtcIntersect1(S, &context, &rayhit3);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit3.ray.tfar == Approx(3));
          CHECK(rayhit3.hit.Ng_x == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit3.hit.Ng_y == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit3.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y min") {
          // corner at x_min, y_min
          Vec3r<T> org4{0.f, 0.f, 4.f};
          Vec3r<T> dir4{0.f, 0.f, -1.f};
          RTCRay ray4{org4[0], org4[1], org4[2], 0, dir4[0], dir4[1], dir4[2], 0, std::numeric_limits<float>::max(),
              0, 0, 0};
          RTCHit hit4;
          hit4.geomID = RTC_INVALID_GEOMETRY_ID;
          hit4.instID[0] = RTC_INVALID_GEOMETRY_ID;
          RTCRayHit rayhit4{ray4, hit4};
          rtcIntersect1(S, &context, &rayhit4);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          CHECK(rayhit4.ray.tfar == Approx(3));
          CHECK(rayhit4.hit.Ng_x == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit4.hit.Ng_y == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit4.hit.Ng_z == Approx(1.f / std::sqrt(3)));
        }
      }
    }
  }
}