//
// Created by ogarten on 18/05/2020.
//


#include <third_party/doctest/doctest/doctest.h>
#include <blazert/embree/primitives/EmbreeCylinder.h>
#include <blazert/embree/scene.h>

#include "../test_helpers.h"

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("EmbreeCylinder", T, float) {
  auto D = rtcNewDevice("verbose=0,start_threads=1,threads=4,set_affinity=1");
  auto S = rtcNewScene(D);
  SUBCASE("bounding box") {
    SUBCASE("center at origin") {
      Vec3r<T> center{0.f, 0.f, 0.f};
      T semi_axis_a = 1;
      T semi_axis_b = 1;
      T height = 2;
      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-1));
        CHECK(bound.lower_y == Approx(-1));
        CHECK(bound.lower_z == Approx(-1));
        CHECK(bound.upper_x == Approx(1));
        CHECK(bound.upper_y == Approx(1));
        CHECK(bound.upper_z == Approx(1));
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-1));
        CHECK(bound.lower_y == Approx(-1));
        CHECK(bound.lower_z == Approx(-1));
        CHECK(bound.upper_x == Approx(1));
        CHECK(bound.upper_y == Approx(1));
        CHECK(bound.upper_z == Approx(1));
      }
    }
    SUBCASE("shifted center") {
      Vec3r<T> center{0.f, 1.f, 4.f};
      T semi_axis_a = 1;
      T semi_axis_b = 1;
      T height = 2;
      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-1));
        CHECK(bound.lower_y == Approx(0));
        CHECK(bound.lower_z == Approx(3));
        CHECK(bound.upper_x == Approx(1));
        CHECK(bound.upper_y == Approx(2));
        CHECK(bound.upper_z == Approx(5));
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
        rtcCommitScene(S);

        RTCBounds bound;
        rtcGetSceneBounds(S, &bound);

        CHECK(bound.lower_x == Approx(-1));
        CHECK(bound.lower_y == Approx(0));
        CHECK(bound.lower_z == Approx(3));
        CHECK(bound.upper_x == Approx(1));
        CHECK(bound.upper_y == Approx(2));
        CHECK(bound.upper_z == Approx(5));
      }
    }
  }
  SUBCASE("intersections") {
    SUBCASE("center at origin") {
      Vec3r<T> center{0.f, 0.f, 0.f};
      T semi_axis_a = 1;
      T semi_axis_b = 2;
      T height = 2;
      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        SUBCASE("origin above") {
          SUBCASE("perpendicular incidence on top") {
            Vec3r<T> org1{0.f, 0.f, 6.5f};
            Vec3r<T> dir1{0.f, 0.f, -1.f};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(5.5));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(1.f));
          }
          SUBCASE("oblique incidence on top") {
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            Vec3r<T> org1{5.f, 0.f, 6.f};
            Vec3r<T> dir1{-1.f, 0.f, -1.f};
            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(50)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(1.f));
          }
          SUBCASE("oblique incidence on shell 1") {
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            Vec3r<T> org1{5, 0, 4};
            Vec3r<T> dir1{-1, 0, -1};
            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 2") {
            Vec3r<T> org1{-5, 0, 4};
            Vec3r<T> dir1{1, 0, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(-1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 3") {
            Vec3r<T> org1{0, 5, 3};
            Vec3r<T> dir1{0, -1, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 4") {
            Vec3r<T> org1{0, -5, 3};
            Vec3r<T> dir1{0, 1, -1};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(-1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
        }
        SUBCASE("origin below") {
          SUBCASE("perpendicular incidence on bottom") {
            Vec3r<T> org1{0.f, 0.f, -8.5f};
            Vec3r<T> dir1{0.f, 0.f, 1.f};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(7.5));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(-1.f));
          }
          SUBCASE("oblique incidence on bottom") {
            Vec3r<T> org1{5.f, 0.f, -6.f};
            Vec3r<T> dir1{-1.f, 0.f, 1.f};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(50)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(-1.f));
          }
          SUBCASE("oblique incidence on shell 1") {
            Vec3r<T> org1{5, 0, -4};
            Vec3r<T> dir1{-1, 0, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 2") {
            Vec3r<T> org1{-5, 0, -4};
            Vec3r<T> dir1{1, 0, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(-1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 3") {
            Vec3r<T> org1{0, 5, -3};
            Vec3r<T> dir1{0, -1, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 4") {
            Vec3r<T> org1{0, -5, -3};
            Vec3r<T> dir1{0, 1, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(-1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
        }
        SUBCASE("origin around shell") {
          SUBCASE("perpendicular incidence") {
            SUBCASE("origin: x+") {
              Vec3r<T> org1{5, 0, 0};
              Vec3r<T> dir1{-1, 0, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(4));
              CHECK(rayhit.hit.Ng_x == Approx(1.f));
              CHECK(rayhit.hit.Ng_y == Approx(0.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
            SUBCASE("origin: x-") {
              Vec3r<T> org1{-5, 0, 0};
              Vec3r<T> dir1{1, 0, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(4));
              CHECK(rayhit.hit.Ng_x == Approx(-1.f));
              CHECK(rayhit.hit.Ng_y == Approx(0.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
            SUBCASE("origin: y+") {
              Vec3r<T> org1{0, 5, 0};
              Vec3r<T> dir1{0, -1, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(3));
              CHECK(rayhit.hit.Ng_x == Approx(0.f));
              CHECK(rayhit.hit.Ng_y == Approx(1.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
            SUBCASE("origin: y-") {
              Vec3r<T> org1{0, -5, 0};
              Vec3r<T> dir1{0, 1, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(3));
              CHECK(rayhit.hit.Ng_x == Approx(0.f));
              CHECK(rayhit.hit.Ng_y == Approx(-1.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
          }
        }
        SUBCASE("origin inside of cylinder") {
          SUBCASE("hit top") {
            Vec3r<T> org1{0, 0, 0};
            Vec3r<T> dir1{0, 0, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(1.f));
          }
          SUBCASE("hit bottom") {
            Vec3r<T> org1{0, 0, 0};
            Vec3r<T> dir1{0, 0, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(-1.f));
          }
          SUBCASE("hit shell 1") {
            Vec3r<T> org1{0, 0, 0};
            Vec3r<T> dir1{-1, 0, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(-1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("hit shell 2") {
            Vec3r<T> org1{0, 0, 0};
            Vec3r<T> dir1{1, 0, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("hit shell 3") {
            Vec3r<T> org1{0, 0, 0};
            Vec3r<T> dir1{0, -1, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(2));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(-1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("hit shell 4") {
            Vec3r<T> org1{0, 0, 0};
            Vec3r<T> dir1{0, 1, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(2));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
        }
      }
    }
    SUBCASE("shifted center") {
      Vec3r<T> center{1.f, 2.f, 0.f};
      T semi_axis_a = 1;
      T semi_axis_b = 2;
      T height = 2;
      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        SUBCASE("origin above") {
          SUBCASE("perpendicular incidence on top") {
            Vec3r<T> org1{1.f, 2.f, 6.5f};
            Vec3r<T> dir1{0.f, 0.f, -1.f};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(5.5));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(1.f));
          }
          SUBCASE("oblique incidence on top") {
            Vec3r<T> org1{6.f, 2.f, 6.f};
            Vec3r<T> dir1{-1.f, 0.f, -1.f};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(50)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(1.f));
          }
          SUBCASE("oblique incidence on shell 1") {
            Vec3r<T> org1{6, 2, 4};
            Vec3r<T> dir1{-1, 0, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 2") {
            Vec3r<T> org1{-4, 2, 4};
            Vec3r<T> dir1{1, 0, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(-1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 3") {
            Vec3r<T> org1{1, 7, 3};
            Vec3r<T> dir1{0, -1, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 4") {
            Vec3r<T> org1{1, -3, 3};
            Vec3r<T> dir1{0, 1, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(-1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
        }
        SUBCASE("origin below") {
          SUBCASE("perpendicular incidence on bottom") {
            Vec3r<T> org1{1.f, 2.f, -8.5f};
            Vec3r<T> dir1{0.f, 0.f, 1.f};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(7.5));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(-1.f));
          }
          SUBCASE("oblique incidence on bottom") {
            Vec3r<T> org1{6.f, 2.f, -6.f};
            Vec3r<T> dir1{-1.f, 0.f, 1.f};

            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(50)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(-1.f));
          }
          SUBCASE("oblique incidence on shell 1") {
            Vec3r<T> org1{6, 2, -4};
            Vec3r<T> dir1{-1, 0, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 2") {
            Vec3r<T> org1{-4, 2, -4};
            Vec3r<T> dir1{1, 0, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(32)));
            CHECK(rayhit.hit.Ng_x == Approx(-1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 3") {
            Vec3r<T> org1{1, 7, -3};
            Vec3r<T> dir1{0, -1, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("oblique incidence on shell 4") {
            Vec3r<T> org1{1, -3, -3};
            Vec3r<T> dir1{0, 1, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(std::sqrt(18)));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(-1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
        }
        SUBCASE("origin around shell") {
          SUBCASE("perpendicular incidence") {
            SUBCASE("origin: x+") {
              Vec3r<T> org1{6, 2, 0};
              Vec3r<T> dir1{-1, 0, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(4));
              CHECK(rayhit.hit.Ng_x == Approx(1.f));
              CHECK(rayhit.hit.Ng_y == Approx(0.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
            SUBCASE("origin: x-") {
              Vec3r<T> org1{-4, 2, 0};
              Vec3r<T> dir1{1, 0, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(4));
              CHECK(rayhit.hit.Ng_x == Approx(-1.f));
              CHECK(rayhit.hit.Ng_y == Approx(0.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
            SUBCASE("origin: y+") {
              Vec3r<T> org1{1, 7, 0};
              Vec3r<T> dir1{0, -1, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(3));
              CHECK(rayhit.hit.Ng_x == Approx(0.f));
              CHECK(rayhit.hit.Ng_y == Approx(1.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
            SUBCASE("origin: y-") {
              Vec3r<T> org1{1, -3, 0};
              Vec3r<T> dir1{0, 1, 0};
              EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
              rtcCommitScene(S);

              RTCIntersectContext context;
              rtcInitIntersectContext(&context);

              RTCRay ray{
                  org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
              RTCHit hit;
              hit.geomID = RTC_INVALID_GEOMETRY_ID;
              hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
              RTCRayHit rayhit{ray, hit};
              rtcIntersect1(S, &context, &rayhit);

              CHECK(rayhit.hit.primID == 0);
              CHECK(rayhit.ray.tfar == Approx(3));
              CHECK(rayhit.hit.Ng_x == Approx(0.f));
              CHECK(rayhit.hit.Ng_y == Approx(-1.f));
              CHECK(rayhit.hit.Ng_z == Approx(0.f));
            }
          }
        }
        SUBCASE("origin inside of cylinder") {
          SUBCASE("hit top") {
            Vec3r<T> org1{1, 2, 0};
            Vec3r<T> dir1{0, 0, 1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(1.f));
          }
          SUBCASE("hit bottom") {
            Vec3r<T> org1{1, 2, 0};
            Vec3r<T> dir1{0, 0, -1};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(-1.f));
          }
          SUBCASE("hit shell 1") {
            Vec3r<T> org1{1, 2, 0};
            Vec3r<T> dir1{-1, 0, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(-1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("hit shell 2") {
            Vec3r<T> org1{1, 2, 0};
            Vec3r<T> dir1{1, 0, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(1));
            CHECK(rayhit.hit.Ng_x == Approx(1.f));
            CHECK(rayhit.hit.Ng_y == Approx(0.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("hit shell 3") {
            Vec3r<T> org1{1, 2, 0};
            Vec3r<T> dir1{0, -1, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(2));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(-1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
          SUBCASE("hit shell 4") {
            Vec3r<T> org1{1, 2, 0};
            Vec3r<T> dir1{0, 1, 0};
            EmbreeCylinder cylinder(D, S, center, semi_axis_a, semi_axis_b, height, rot);
            rtcCommitScene(S);

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);

            RTCRay ray{
                org1[0], org1[1], org1[2], 0, dir1[0], dir1[1], dir1[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};
            RTCHit hit;
            hit.geomID = RTC_INVALID_GEOMETRY_ID;
            hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            RTCRayHit rayhit{ray, hit};
            rtcIntersect1(S, &context, &rayhit);

            CHECK(rayhit.hit.primID == 0);
            CHECK(rayhit.ray.tfar == Approx(2));
            CHECK(rayhit.hit.Ng_x == Approx(0.f));
            CHECK(rayhit.hit.Ng_y == Approx(1.f));
            CHECK(rayhit.hit.Ng_z == Approx(0.f));
          }
        }
      }
    }
  }
}
