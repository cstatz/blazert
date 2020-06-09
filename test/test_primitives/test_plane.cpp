//
// Created by ogarten on 13/05/2020.
//
/*
#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/plane.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include <third_party/doctest/doctest/doctest.h>
#include "../test_helpers.h"

using namespace blazert;
using namespace doctest;


TEST_CASE_TEMPLATE("Plane", TestType, float, double) {
  const TestType d1 = 2.f;
  const TestType d2 = 2.f;

  auto centers = std::make_unique<Vec3rList<TestType>>();
  auto dxx = std::make_unique<std::vector<TestType>>();
  auto dyy = std::make_unique<std::vector<TestType>>();
  auto rotations = std::make_unique<Mat3rList<TestType>>();

  dxx->push_back(d1);
  dyy->push_back(d2);

  SUBCASE("bounding box") {
    SUBCASE("center at origin") {
      const Vec3r<TestType> center{0.f, 0.f, 0.f};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        const Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(-1.f));
        CHECK(bmin[1] == Approx(-1.f));
        CHECK(bmin[2] == Approx(-std::numeric_limits<TestType>::min()));
        CHECK(bmax[0] == Approx(1.f));
        CHECK(bmax[1] == Approx(1.f));
        CHECK(bmax[2] == Approx(std::numeric_limits<TestType>::min()));
      }
      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(-std::numeric_limits<TestType>::min()));
        CHECK(bmin[1] == Approx(-1.f));
        CHECK(bmin[2] == Approx(-1.f));
        CHECK(bmax[0] == Approx(std::numeric_limits<TestType>::min()));
        CHECK(bmax[1] == Approx(1.f));
        CHECK(bmax[2] == Approx(1.f));
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {

        // plane on z=0, later rotated to x = 0
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<TestType> axis{static_cast<TestType>(1 / std::sqrt(2)), static_cast<TestType>(1 / std::sqrt(2)), 0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(-1.f));
        CHECK(bmin[1] == Approx(-1.f));
        CHECK(bmin[2] == Approx(-sqrt(2)));
        CHECK(bmax[0] == Approx(1.f));
        CHECK(bmax[1] == Approx(1.f));
        CHECK(bmax[2] == Approx(sqrt(2)));
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<TestType> axis{0, 0, 1};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 4);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(-sqrt(2)));
        CHECK(bmin[1] == Approx(-sqrt(2)));
        CHECK(bmin[2] == Approx(0));
        CHECK(bmax[0] == Approx(sqrt(2)));
        CHECK(bmax[1] == Approx(sqrt(2)));
        CHECK(bmax[2] == Approx(0));
      }
    }
    SUBCASE("shifted center") {
      const Vec3r<TestType> center{1.f, 1.f, 1.f};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        const Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(0.f));
        CHECK(bmin[1] == Approx(0.f));
        CHECK(bmin[2] == Approx(1.f));
        CHECK(bmax[0] == Approx(2.f));
        CHECK(bmax[1] == Approx(2.f));
        CHECK(bmax[2] == Approx(1.f));
      }
      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(1.f));
        CHECK(bmin[1] == Approx(0.f));
        CHECK(bmin[2] == Approx(0.f));
        CHECK(bmax[0] == Approx(1.f));
        CHECK(bmax[1] == Approx(2.f));
        CHECK(bmax[2] == Approx(2.f));
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {
        // plane on z=0, rotated such that (xmin, ymin) corner is parallel to z-axis
        const Vec3r<TestType> axis{static_cast<TestType>(1 / std::sqrt(2)), static_cast<TestType>(1 / std::sqrt(2)), 0};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 2);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(0));
        CHECK(bmin[1] == Approx(0));
        CHECK(bmin[2] == Approx(1 - std::sqrt(2)));
        CHECK(bmax[0] == Approx(2));
        CHECK(bmax[1] == Approx(2));
        CHECK(bmax[2] == Approx(1 + std::sqrt(2)));
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<TestType> axis{0, 0, 1};
        Mat3r<TestType> rot = arbitraryRotationMatrix(axis, pi<TestType> / 4);
        rotations->push_back(rot);

        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        Vec3r<TestType> bmin, bmax;
        planes.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(1 - std::sqrt(2)));
        CHECK(bmin[1] == Approx(1 - std::sqrt(2)));
        CHECK(bmin[2] == Approx(1));
        CHECK(bmax[0] == Approx(1 + std::sqrt(2)));
        CHECK(bmax[1] == Approx(1 + std::sqrt(2)));
        CHECK(bmax[2] == Approx(1));
      }
    }
  }
  SUBCASE("intersections") {
    SUBCASE("center at origin") {
      const Vec3r<TestType> center{0.f, 0.f, 0.f};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
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

        CHECK(hit_plane);
        CHECK(rayhit.prim_id == 0);
        CHECK(rayhit.hit_distance == Approx(5));
        CHECK(rayhit.normal[0] == Approx(0.f));
        CHECK(rayhit.normal[1] == Approx(0.f));
        CHECK(rayhit.normal[2] == Approx(1.f));
      }
      SUBCASE("rotated about (0,1,0)") {
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

        CHECK(hit_plane);
        CHECK(rayhit.prim_id == 0);
        CHECK(rayhit.hit_distance == Approx(5));
        CHECK(rayhit.normal[0] == Approx(1.f));
        CHECK(rayhit.normal[1] == Approx(0.f));
        CHECK(rayhit.normal[2] == Approx(0.f));
      }
      SUBCASE("rotated about normalized(1,1,0), edge hit") {
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
        CHECK(hit_plane);
        CHECK(rayhit.prim_id == 0);
        CHECK(rayhit.hit_distance == Approx(5 * std::sqrt(2)));
        CHECK(rayhit.normal[0] == Approx(-1 / std::sqrt(2)));
        CHECK(rayhit.normal[1] == Approx(1 / std::sqrt(2)));
        CHECK(rayhit.normal[2] == Approx(0.f));// for rotated planes, you can expect small numerical error
        // of size 1e16
      }
      SUBCASE("non-rotated, ray origin outside plane bounds") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        SUBCASE("outside on positive x-axis") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4 * std::sqrt(2.f)));
          CHECK(rayhit.normal[0] == Approx(0.f));
          CHECK(rayhit.normal[1] == Approx(0.f));
          CHECK(rayhit.normal[2] == Approx(1.f));
        }
        SUBCASE("outside on negative x-axis") {
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
          CHECK(hit_plane);
          CHECK(rayhit2.prim_id == 0);
          CHECK(rayhit2.hit_distance == Approx(4. * std::sqrt(2.f)));
          CHECK(rayhit2.normal[0] == Approx(0.f));
          CHECK(rayhit2.normal[1] == Approx(0.f));
          CHECK(rayhit2.normal[2] == Approx(-1.f));
        }
      }
      SUBCASE("non-rotated, edge intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SUBCASE("edge: x max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit.normal[1] == Approx(0.f));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: x_min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit.normal[1] == Approx(0.f));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(0.f));
          CHECK(rayhit.normal[1] == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(0.f));
          CHECK(rayhit.normal[1] == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
      }
      SUBCASE("non-rotated, corner intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SUBCASE("corner: x max, y max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x max, y min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4));
          CHECK(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
      }
    }
    SUBCASE("shifted center") {
      const Vec3r<TestType> center{1.f, 1.f, 1.f};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
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

        CHECK(hit_plane);
        CHECK(rayhit.prim_id == 0);
        CHECK(rayhit.hit_distance == Approx(4));
        CHECK(rayhit.normal[0] == Approx(0.f));
        CHECK(rayhit.normal[1] == Approx(0.f));
        CHECK(rayhit.normal[2] == Approx(1.f));
      }
      SUBCASE("rotated, non-shifted") {
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

        CHECK(hit_plane);
        CHECK(rayhit.prim_id == 0);
        CHECK(rayhit.hit_distance == Approx(4));
        CHECK(rayhit.normal[0] == Approx(1.f));
        CHECK(rayhit.normal[1] == Approx(0.f));
        CHECK(rayhit.normal[2] == Approx(0.f));
      }
      SUBCASE("rotated about r=normalized(1,1,0), edge hit") {
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
        CHECK(hit_plane);
        CHECK(rayhit.prim_id == 0);
        CHECK(rayhit.hit_distance == Approx(5 * std::sqrt(2)));
        CHECK(rayhit.normal[0] == Approx(-1 / std::sqrt(2)));
        CHECK(rayhit.normal[1] == Approx(1 / std::sqrt(2)));
        CHECK(rayhit.normal[2] == Approx(0.f));// for rotated planes, you can expect small numerical error
        // of size 1e16
      }
      SUBCASE("non-rotated, ray origin outside plane bounds") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        SUBCASE("outside on positive x-axis") {
          Vec3r<TestType> org1{5.f, 1.f, 5.f};
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(4 * std::sqrt(2.f)));
          CHECK(rayhit.normal[0] == Approx(0.f));
          CHECK(rayhit.normal[1] == Approx(0.f));
          CHECK(rayhit.normal[2] == Approx(1.f));
        }
        SUBCASE("outside on negative x-axis") {
          Vec3r<TestType> org2{-3.f, 1.f, -3.f};
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
          CHECK(hit_plane);
          CHECK(rayhit2.prim_id == 0);
          CHECK(rayhit2.hit_distance == Approx(4. * std::sqrt(2.f)));
          CHECK(rayhit2.normal[0] == Approx(0.f));
          CHECK(rayhit2.normal[1] == Approx(0.f));
          CHECK(rayhit2.normal[2] == Approx(-1.f));
        }
      }
      SUBCASE("non-rotated, edge intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SUBCASE("edge: x max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit.normal[1] == Approx(0.f));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: x_min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit.normal[1] == Approx(0.f));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(0.f));
          CHECK(rayhit.normal[1] == Approx(1.f / std::sqrt(2)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
        SUBCASE("edge: y min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(0.f));
          CHECK(rayhit.normal[1] == Approx(-1.f / std::sqrt(2)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(2)));
        }
      }
      SUBCASE("non-rotated, corner intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<TestType> rot = blaze::IdentityMatrix<TestType>(3UL);
        rotations->push_back(rot);
        Plane<TestType> planes(*centers, *dxx, *dyy, *rotations);

        BVHTraceOptions<TestType> trace_options;
        PlaneIntersector<TestType> plane_intersector{*centers, *dxx, *dyy, *rotations};
        RayHit<TestType> rayhit;
        SUBCASE("corner: x max, y max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y max") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x max, y min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
        SUBCASE("corner: x min, y min") {
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
          CHECK(hit_plane);
          CHECK(rayhit.prim_id == 0);
          CHECK(rayhit.hit_distance == Approx(3));
          CHECK(rayhit.normal[0] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[1] == Approx(-1.f / std::sqrt(3)));
          CHECK(rayhit.normal[2] == Approx(1.f / std::sqrt(3)));
        }
      }
    }
  }
}
 */