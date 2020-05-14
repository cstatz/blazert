//
// Created by ogarten on 13/05/2020.
//
#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/plane.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../catch.hpp"
#include "../test_helpers.h"

using namespace blazert;

TEMPLATE_TEST_CASE("Plane", "[bounding box,  intersections]", float, double) {
  const Vec3r<TestType> center{0.f, 0.f, 0.f};
  const TestType d1 = 2.f;
  const TestType d2 = 2.f;

  auto centers = std::make_unique<Vec3rList<TestType>>();
  auto dxx = std::make_unique<std::vector<TestType>>();
  auto dyy = std::make_unique<std::vector<TestType>>();
  auto rotations = std::make_unique<Mat3rList<TestType>>();

  dxx->push_back(d1);
  dyy->push_back(d2);

  SECTION("bounding box") {
    SECTION("center at origin") {
      const Vec3r<TestType> center{0.f, 0.f, 0.f};
      centers->emplace_back(center);

      SECTION("non-rotated") {
        const Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(-1.f));
        REQUIRE(bmin[1] == Approx(-1.f));
        REQUIRE(bmin[2] == Approx(-std::numeric_limits<TestType>::min()));
        REQUIRE(bmax[0] == Approx(1.f));
        REQUIRE(bmax[1] == Approx(1.f));
        REQUIRE(bmax[2] == Approx(std::numeric_limits<TestType>::min()));
      }
      SECTION("Rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(-std::numeric_limits<TestType>::min()));
        REQUIRE(bmin[1] == Approx(-1.f));
        REQUIRE(bmin[2] == Approx(-1.f));
        REQUIRE(bmax[0] == Approx(std::numeric_limits<TestType>::min()));
        REQUIRE(bmax[1] == Approx(1.f));
        REQUIRE(bmax[2] == Approx(1.f));
      }
      SECTION("rotated about r=normalized(1,1,0)") {

        // plane on z=0, later rotated to x = 0
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<TestType> axis{static_cast<TestType>(1 / std::sqrt(2)), static_cast<TestType>(1 / std::sqrt(2)), 0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(-1.f));
        REQUIRE(bmin[1] == Approx(-1.f));
        REQUIRE(bmin[2] == Approx(-sqrt(2)));
        REQUIRE(bmax[0] == Approx(1.f));
        REQUIRE(bmax[1] == Approx(1.f));
        REQUIRE(bmax[2] == Approx(sqrt(2)));
      }
      SECTION("rotated in xy-plane") {
        const Vec3r<TestType> axis{0, 0, 1};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 4);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(-sqrt(2)));
        REQUIRE(bmin[1] == Approx(-sqrt(2)));
        REQUIRE(bmin[2] == Approx(0).margin(0.0000001));
        REQUIRE(bmax[0] == Approx(sqrt(2)));
        REQUIRE(bmax[1] == Approx(sqrt(2)));
        REQUIRE(bmax[2] == Approx(0).margin(0.0000001));
      }
    }
    SECTION("shifted center") {
      const Vec3r<TestType> center{1.f, 1.f, 1.f};
      centers->emplace_back(center);

      SECTION("non-rotated") {
        const Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(0.f));
        REQUIRE(bmin[1] == Approx(0.f));
        REQUIRE(bmin[2] == Approx(1.f));
        REQUIRE(bmax[0] == Approx(2.f));
        REQUIRE(bmax[1] == Approx(2.f));
        REQUIRE(bmax[2] == Approx(1.f));
      }
      SECTION("Rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(1.f));
        REQUIRE(bmin[1] == Approx(0.f));
        REQUIRE(bmin[2] == Approx(0.f));
        REQUIRE(bmax[0] == Approx(1.f));
        REQUIRE(bmax[1] == Approx(2.f));
        REQUIRE(bmax[2] == Approx(2.f));
      }
      SECTION("rotated about r=normalized(1,1,0)") {
        // plane on z=0, rotated such that (xmin, ymin) corner is parallel to z-axis
        const Vec3r<TestType> axis{static_cast<TestType>(1 / std::sqrt(2)), static_cast<TestType>(1 / std::sqrt(2)), 0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(0).margin(std::numeric_limits<TestType>::epsilon()));
        REQUIRE(bmin[1] == Approx(0).margin(std::numeric_limits<TestType>::epsilon()));
        REQUIRE(bmin[2] == Approx(1 - std::sqrt(2)));
        REQUIRE(bmax[0] == Approx(2));
        REQUIRE(bmax[1] == Approx(2));
        REQUIRE(bmax[2] == Approx(1 + std::sqrt(2)));
      }
      SECTION("rotated in xy-plane") {
        const Vec3r<TestType> axis{0, 0, 1};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 4);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        REQUIRE(bmin[0] == Approx(1 - std::sqrt(2)));
        REQUIRE(bmin[1] == Approx(1 - std::sqrt(2)));
        REQUIRE(bmin[2] == Approx(1));
        REQUIRE(bmax[0] == Approx(1 + std::sqrt(2)));
        REQUIRE(bmax[1] == Approx(1 + std::sqrt(2)));
        REQUIRE(bmax[2] == Approx(1));
      }
    }
  }
  SECTION("intersections") {
    SECTION("center at origin") {
      const Vec3r<TestType> center{0.f, 0.f, 0.f};
      centers->emplace_back(center);

      SECTION("non-rotated") {
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<TestType> org1{0.f, 0.f, 5.f};
        const Vec3r<TestType> dir1{0.f, 0.f, -1.f};
        const Ray<TestType> ray{org1, dir1};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};

        // Test intersections
        update_intersector(plane_intersector, ray.max_hit_distance, -1);
        prepare_traversal(plane_intersector, ray, trace_options);
        TestType hit_distance = plane_intersector.hit_distance;
        const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
        update_intersector(plane_intersector, hit_distance, 0);
        post_traversal(plane_intersector, ray, hit_plane, rayhit);

        REQUIRE(hit_plane);
        REQUIRE(rayhit.prim_id == 0);
        REQUIRE(rayhit.hit_distance == Approx(5));
        REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
        REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
        REQUIRE(rayhit.normal[2] == Approx(1.f));
      }
      SECTION("rotated, non-shifted") {
        // matrix which rotates the plane 45 degrees about the z-axis ( x = 0 is now
        // plane eq)
        const Vec3r<TestType> axis{0, 1, 0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);

        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> org1{5.f, 0.f, 0.f};
        Vec3r<TestType> dir1{-1.f, 0.f, 0.f};
        const Ray<TestType> ray{org1, dir1};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};

        // Test intersections
        update_intersector(plane_intersector, ray.max_hit_distance, -1);
        prepare_traversal(plane_intersector, ray, trace_options);
        TestType hit_distance = plane_intersector.hit_distance;
        const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
        update_intersector(plane_intersector, hit_distance, 0);
        post_traversal(plane_intersector, ray, hit_plane, rayhit);

        REQUIRE(hit_plane);
        REQUIRE(rayhit.prim_id == 0);
        REQUIRE(rayhit.hit_distance == Approx(5));
        REQUIRE(rayhit.normal[0] == Approx(1.f));
        REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
        REQUIRE(rayhit.normal[2] == Approx(0.f).margin(0.0001));
      }
      SECTION("rotated about r=normalized(1,1,0), edge hit") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<TestType> axis{static_cast<TestType>(1. / std::sqrt(2.)),
                                   static_cast<TestType>(1. / std::sqrt(2.)),
                                   0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<TestType> org1{-5.f, 5.f, 0.5f};
        const Vec3r<TestType> dir1{1.f, -1.f, 0.f};
        const Ray<TestType> ray{org1, dir1};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};

        // Test intersections
        update_intersector(plane_intersector, ray.max_hit_distance, -1);
        prepare_traversal(plane_intersector, ray, trace_options);
        TestType hit_distance = plane_intersector.hit_distance;
        const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
        update_intersector(plane_intersector, hit_distance, 0);
        post_traversal(plane_intersector, ray, hit_plane, rayhit);

        // hits from the negative x direction -> normal vector should point towards
        // that direction
        REQUIRE(hit_plane);
        REQUIRE(rayhit.prim_id == 0);
        REQUIRE(rayhit.hit_distance == Approx(5 * std::sqrt(2)));
        REQUIRE(rayhit.normal[0] == Approx(-1 / std::sqrt(2)));
        REQUIRE(rayhit.normal[1] == Approx(1 / std::sqrt(2)));
        REQUIRE(rayhit.normal[2] == Approx(0.f).margin(0.0001));// for rotated planes, you can expect small numerical error
        // of size 1e16
      }
      SECTION("non-rotated, non-shifted, ray origin outside plane bounds") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        SECTION("outside on positive x-axis") {
          Vec3r<TestType> org1{4.f, 0.f, 4.f};
          Vec3r<TestType> dir1{-1.f, 0.f, -1.f};
          const Ray<TestType> ray{org1, dir1};
          RayHit<TestType> rayhit;
          // Test intersections
          update_intersector(plane_intersector, ray.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4 * std::sqrt(2.f)));
          REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[2] == Approx(1.f));
        }
        SECTION("outside on negative x-axis") {
          Vec3r<TestType> org2{-4.f, 0.f, -4.f};
          Vec3r<TestType> dir2{1.f, 0.f, 1.f};
          const Ray<TestType> ray2{org2, dir2};
          RayHit<TestType> rayhit2;

          // Test intersections
          update_intersector(plane_intersector, ray2.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray2, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray2, hit_plane, rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit2.prim_id == 0);
          REQUIRE(rayhit2.hit_distance == Approx(4. * std::sqrt(2.f)));
          REQUIRE(rayhit2.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit2.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit2.normal[2] == Approx(-1.f));
        }
      }
      SECTION("non-rotated, non-shifted, edge intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SECTION("edge: x max") {
          // edge at x_max
          Vec3r<TestType> org1{1.f, 0.f, 4.f};
          Vec3r<TestType> dir1{0.f, 0.f, -1.f};
          const Ray<TestType> ray{org1, dir1};

          // Test intersections
          update_intersector(plane_intersector, ray.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SECTION("edge: x_min") {
          // edge at x_min
          Vec3r<TestType> org2{-1.f, 0.f, 4.f};
          Vec3r<TestType> dir2{0.f, 0.f, -1.f};
          const Ray<TestType> ray2{org2, dir2};

          // Test intersections
          update_intersector(plane_intersector, ray2.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray2, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray2, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(-1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SECTION("edge: y max") {
          Vec3r<TestType> org3{0.f, 1.f, 4.f};
          Vec3r<TestType> dir3{0.f, 0.f, -1.f};
          const Ray<TestType> ray3{org3, dir3};

          // Test intersections
          update_intersector(plane_intersector, ray3.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray3, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray3, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[1] == Approx(1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SECTION("edge: y min") {
          // edge at y_min
          Vec3r<TestType> org4{0.f, -1.f, 4.f};
          Vec3r<TestType> dir4{0.f, 0.f, -1.f};
          const Ray<TestType> ray4{org4, dir4};

          // Test intersections
          update_intersector(plane_intersector, ray4.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray4, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray4, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[1] == Approx(-1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
      }
      SECTION("non-rotated, non-shifted, corner intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SECTION("corner: x max, y max") {
          Vec3r<TestType> org1{1.f, 1.f, 4.f};
          Vec3r<TestType> dir1{0.f, 0.f, -1.f};
          const Ray<TestType> ray{org1, dir1};

          // Test intersections
          update_intersector(plane_intersector, ray.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SECTION("corner: x min, y max") {
          Vec3r<TestType> org2{-1.f, 1.f, 4.f};
          Vec3r<TestType> dir2{0.f, 0.f, -1.f};
          const Ray<TestType> ray2{org2, dir2};

          // Test intersections
          update_intersector(plane_intersector, ray2.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray2, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray2, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SECTION("corner: x max, y min") {
          Vec3r<TestType> org3{1.f, -1.f, 4.f};
          Vec3r<TestType> dir3{0.f, 0.f, -1.f};
          const Ray<TestType> ray3{org3, dir3};

          // Test intersections
          update_intersector(plane_intersector, ray3.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray3, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray3, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SECTION("corner: x min, y min") {
          // corner at x_min, y_min
          Vec3r<TestType> org4{-1.f, -1.f, 4.f};
          Vec3r<TestType> dir4{0.f, 0.f, -1.f};
          const Ray<TestType> ray4{org4, dir4};

          // Test intersections
          update_intersector(plane_intersector, ray4.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray4, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray4, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(4));
          REQUIRE(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
      }
    }
    SECTION("shifted center") {
      const Vec3r<TestType> center{1.f, 1.f, 1.f};
      centers->emplace_back(center);

      SECTION("non-rotated") {
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<TestType> org1{1.f, 1.f, 5.f};
        const Vec3r<TestType> dir1{0.f, 0.f, -1.f};
        const Ray<TestType> ray{org1, dir1};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};

        // Test intersections
        update_intersector(plane_intersector, ray.max_hit_distance, -1);
        prepare_traversal(plane_intersector, ray, trace_options);
        TestType hit_distance = plane_intersector.hit_distance;
        const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
        update_intersector(plane_intersector, hit_distance, 0);
        post_traversal(plane_intersector, ray, hit_plane, rayhit);

        REQUIRE(hit_plane);
        REQUIRE(rayhit.prim_id == 0);
        REQUIRE(rayhit.hit_distance == Approx(4));
        REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
        REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
        REQUIRE(rayhit.normal[2] == Approx(1.f));
      }
      SECTION("rotated, non-shifted") {
        // matrix which rotates the plane 90 degrees about the y-axis ( x = 0 is now
        // plane eq)
        const Vec3r<TestType> axis{0, 1, 0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);

        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> org1{5.f, 1.f, 1.f};
        Vec3r<TestType> dir1{-1.f, 0.f, 0.f};
        const Ray<TestType> ray{org1, dir1};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};

        // Test intersections
        update_intersector(plane_intersector, ray.max_hit_distance, -1);
        prepare_traversal(plane_intersector, ray, trace_options);
        TestType hit_distance = plane_intersector.hit_distance;
        const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
        update_intersector(plane_intersector, hit_distance, 0);
        post_traversal(plane_intersector, ray, hit_plane, rayhit);

        REQUIRE(hit_plane);
        REQUIRE(rayhit.prim_id == 0);
        REQUIRE(rayhit.hit_distance == Approx(4));
        REQUIRE(rayhit.normal[0] == Approx(1.f));
        REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
        REQUIRE(rayhit.normal[2] == Approx(0.f).margin(0.0001));
      }
      SECTION("rotated about r=normalized(1,1,0), edge hit") {
        // matrix which rotates the plane about axis = normalized(1,1,0)
        const Vec3r<TestType> axis{static_cast<TestType>(1. / std::sqrt(2.)),
                                   static_cast<TestType>(1. / std::sqrt(2.)),
                                   0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<TestType> org1{-4.f, 6.f, 0.5f};
        const Vec3r<TestType> dir1{1.f, -1.f, 0.f};
        const Ray<TestType> ray{org1, dir1};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};

        // Test intersections
        update_intersector(plane_intersector, ray.max_hit_distance, -1);
        prepare_traversal(plane_intersector, ray, trace_options);
        TestType hit_distance = plane_intersector.hit_distance;
        const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
        update_intersector(plane_intersector, hit_distance, 0);
        post_traversal(plane_intersector, ray, hit_plane, rayhit);

        // hits from the negative x direction -> normal vector should point towards
        // that direction
        REQUIRE(hit_plane);
        REQUIRE(rayhit.prim_id == 0);
        REQUIRE(rayhit.hit_distance == Approx(5 * std::sqrt(2)));
        REQUIRE(rayhit.normal[0] == Approx(-1 / std::sqrt(2)));
        REQUIRE(rayhit.normal[1] == Approx(1 / std::sqrt(2)));
        REQUIRE(rayhit.normal[2] == Approx(0.f).margin(0.0001));// for rotated planes, you can expect small numerical error
        // of size 1e16
      }
      SECTION("non-rotated, non-shifted, ray origin outside plane bounds") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        SECTION("outside on positive x-axis") {
          Vec3r<TestType> org1{5.f, 1.f, 4.f};
          Vec3r<TestType> dir1{-1.f, 0.f, -1.f};
          const Ray<TestType> ray{org1, dir1};
          RayHit<TestType> rayhit;
          // Test intersections
          update_intersector(plane_intersector, ray.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3 * std::sqrt(2.f)));
          REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[2] == Approx(1.f));
        }
        SECTION("outside on negative x-axis") {
          Vec3r<TestType> org2{-3.f, 1.f, -4.f};
          Vec3r<TestType> dir2{1.f, 0.f, 1.f};
          const Ray<TestType> ray2{org2, dir2};
          RayHit<TestType> rayhit2;

          // Test intersections
          update_intersector(plane_intersector, ray2.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray2, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray2, hit_plane, rayhit2);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit2.prim_id == 0);
          REQUIRE(rayhit2.hit_distance == Approx(5. * std::sqrt(2.f)));
          REQUIRE(rayhit2.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit2.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit2.normal[2] == Approx(-1.f));
        }
      }
      SECTION("non-rotated, non-shifted, edge intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SECTION("edge: x max") {
          // edge at x_max
          Vec3r<TestType> org1{2.f, 1.f, 4.f};
          Vec3r<TestType> dir1{0.f, 0.f, -1.f};
          const Ray<TestType> ray{org1, dir1};

          // Test intersections
          update_intersector(plane_intersector, ray.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SECTION("edge: x_min") {
          // edge at x_min
          Vec3r<TestType> org2{0.f, 1.f, 4.f};
          Vec3r<TestType> dir2{0.f, 0.f, -1.f};
          const Ray<TestType> ray2{org2, dir2};

          // Test intersections
          update_intersector(plane_intersector, ray2.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray2, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray2, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(-1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[1] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SECTION("edge: y max") {
          Vec3r<TestType> org3{1.f, 2.f, 4.f};
          Vec3r<TestType> dir3{0.f, 0.f, -1.f};
          const Ray<TestType> ray3{org3, dir3};

          // Test intersections
          update_intersector(plane_intersector, ray3.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray3, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray3, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[1] == Approx(1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SECTION("edge: y min") {
          // edge at y_min
          Vec3r<TestType> org4{1.f, 0.f, 4.f};
          Vec3r<TestType> dir4{0.f, 0.f, -1.f};
          const Ray<TestType> ray4{org4, dir4};

          // Test intersections
          update_intersector(plane_intersector, ray4.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray4, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray4, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(0.f).margin(0.0001));
          REQUIRE(rayhit.normal[1] == Approx(-1.f / std::sqrt(2)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
      }
      SECTION("non-rotated, non-shifted, corner intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SECTION("corner: x max, y max") {
          Vec3r<TestType> org1{2.f, 2.f, 4.f};
          Vec3r<TestType> dir1{0.f, 0.f, -1.f};
          const Ray<TestType> ray{org1, dir1};

          // Test intersections
          update_intersector(plane_intersector, ray.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SECTION("corner: x min, y max") {
          Vec3r<TestType> org2{0.f, 2.f, 4.f};
          Vec3r<TestType> dir2{0.f, 0.f, -1.f};
          const Ray<TestType> ray2{org2, dir2};

          // Test intersections
          update_intersector(plane_intersector, ray2.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray2, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray2, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SECTION("corner: x max, y min") {
          Vec3r<TestType> org3{2.f, 0.f, 4.f};
          Vec3r<TestType> dir3{0.f, 0.f, -1.f};
          const Ray<TestType> ray3{org3, dir3};

          // Test intersections
          update_intersector(plane_intersector, ray3.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray3, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray3, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SECTION("corner: x min, y min") {
          // corner at x_min, y_min
          Vec3r<TestType> org4{0.f, 0.f, 4.f};
          Vec3r<TestType> dir4{0.f, 0.f, -1.f};
          const Ray<TestType> ray4{org4, dir4};

          // Test intersections
          update_intersector(plane_intersector, ray4.max_hit_distance, -1);
          prepare_traversal(plane_intersector, ray4, trace_options);
          TestType hit_distance = plane_intersector.hit_distance;
          const bool hit_plane = intersect(plane_intersector, hit_distance, 0);
          update_intersector(plane_intersector, hit_distance, 0);
          post_traversal(plane_intersector, ray4, hit_plane, rayhit);

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          REQUIRE(hit_plane);
          REQUIRE(rayhit.prim_id == 0);
          REQUIRE(rayhit.hit_distance == Approx(3));
          REQUIRE(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          REQUIRE(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
      }
    }
  }
}