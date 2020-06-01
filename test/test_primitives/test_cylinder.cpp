//
// Created by ogarten on 15/05/2020.
//

#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/cylinder.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include <third_party/doctest/doctest/doctest.h>
#include "../test_helpers.h"

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("cylinder", T, float, double) {
  auto centers = std::make_unique<Vec3rList<T>>();
  auto semi_axes_a = std::make_unique<std::vector<T>>();
  auto semi_axes_b = std::make_unique<std::vector<T>>();
  auto heights = std::make_unique<std::vector<T>>();
  auto rotations = std::make_unique<Mat3rList<T>>();

  SUBCASE("bounding box") {
    SUBCASE("center at origin") {
      centers->emplace_back(Vec3r<T>{0.f, 0.f, 0.f});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(1);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        Cylinder cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        Vec3r<T> bmin, bmax;
        cylinders.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(-1));
        CHECK(bmin[1] == Approx(-1));
        CHECK(bmin[2] == Approx(0));
        CHECK(bmax[0] == Approx(1));
        CHECK(bmax[1] == Approx(1));
        CHECK(bmax[2] == Approx(2));
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);
        Cylinder cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        Vec3r<T> bmin, bmax;
        cylinders.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(0));
        CHECK(bmin[1] == Approx(-1));
        CHECK(bmin[2] == Approx(-1));
        CHECK(bmax[0] == Approx(2));
        CHECK(bmax[1] == Approx(1));
        CHECK(bmax[2] == Approx(1));
      }
    }
    SUBCASE("shifted center") {
      centers->emplace_back(Vec3r<T>{0.f, 1.f, 4.f});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(1);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        Cylinder cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        Vec3r<T> bmin, bmax;
        cylinders.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(-1));
        CHECK(bmin[1] == Approx(0));
        CHECK(bmin[2] == Approx(4));
        CHECK(bmax[0] == Approx(1));
        CHECK(bmax[1] == Approx(2));
        CHECK(bmax[2] == Approx(6));
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);
        Cylinder cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        Vec3r<T> bmin, bmax;
        cylinders.BoundingBox(bmin, bmax, 0);

        CHECK(bmin[0] == Approx(0));
        CHECK(bmin[1] == Approx(0));
        CHECK(bmin[2] == Approx(3));
        CHECK(bmax[0] == Approx(2));
        CHECK(bmax[1] == Approx(2));
        CHECK(bmax[2] == Approx(5));
      }
    }
  }
  SUBCASE("intersections") {
    SUBCASE("center at origin") {
      centers->emplace_back(Vec3r<T>{0.f, 0.f, 0.f});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(2);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        SUBCASE("hits") {
          SUBCASE("origin above") {
            SUBCASE("perpendicular incidence on top") {
              Vec3r<T> org1{0.f, 0.f, 7.5f};
              Vec3r<T> dir1{0.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(5.5));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(1.f));
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{5.f, 0.f, 7.f};
              Vec3r<T> dir1{-1.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(50)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(1.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{5, 0, 5};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, 5};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(-1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, 4};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, 4};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(-1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin below") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{0.f, 0.f, -7.5f};
              Vec3r<T> dir1{0.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(7.5));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(-1.f));
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{5.f, 0.f, -5.f};
              Vec3r<T> dir1{-1.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(50)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(-1.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{5, 0, -3};
              Vec3r<T> dir1{-1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, -3};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(-1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, -2};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, -2};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(-1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin around shell") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{5, 0, 1};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(4));
                CHECK(rayhit.normal[0] == Approx(1.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-5, 0, 1};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(4));
                CHECK(rayhit.normal[0] == Approx(-1.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{0, 5, 1};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(3));
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(1.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{0, -5, 1};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(3));
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(-1.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
            }
          }
          SUBCASE("origin inside of cylinder") {
            SUBCASE("hit top") {
              Vec3r<T> org1{0, 0, 1};
              Vec3r<T> dir1{0, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(1.f));
            }
            SUBCASE("hit bottom") {
              Vec3r<T> org1{0, 0, 1};
              Vec3r<T> dir1{0, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(-1.f));
            }
            SUBCASE("hit shell 1") {
              Vec3r<T> org1{0, 0, 1};
              Vec3r<T> dir1{-1, 0, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(-1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("hit shell 2") {
              Vec3r<T> org1{0, 0, 1};
              Vec3r<T> dir1{1, 0, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("hit shell 3") {
              Vec3r<T> org1{0, 0, 1};
              Vec3r<T> dir1{0, -1, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(2));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(-1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("hit shell 4") {
              Vec3r<T> org1{0, 0, 1};
              Vec3r<T> dir1{0, 1, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(2));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
        }
        SUBCASE("no hits") {
          SUBCASE("origin above (direction inverted)") {
            SUBCASE("perpendicular incidence on top") {

              Vec3r<T> org1{0.f, 0.f, 7.5f};
              Vec3r<T> dir1{0.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{5.f, 0.f, 7.f};
              Vec3r<T> dir1{1.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{5, 0, 5};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, 5};
              Vec3r<T> dir1{-1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, 4};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, 4};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin below (direction inverted)") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{0.f, 0.f, -7.5f};
              Vec3r<T> dir1{0.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{5.f, 0.f, -5.f};
              Vec3r<T> dir1{1.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{5, 0, -3};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, -3};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, -2};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, -2};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin around shell (direction inverted") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{5, 0, 1};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-5, 0, 1};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{0, 5, 1};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{0, -5, 1};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
            }
          }
        }
      }
      SUBCASE("rotated") {
        SUBCASE("hits") {
          SUBCASE("about z-axis") {
            const Vec3r<T> axis{0, 0, 1};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{0.f, 0.f, 7.5f};
            Vec3r<T> dir1{0.f, 0.f, -1.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK(hit_cylinder);
            CHECK(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == Approx(5.5));
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(1.f));
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{7.5f, 0.f, 0.f};
            Vec3r<T> dir1{-1.f, 0.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK(hit_cylinder);
            CHECK(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == Approx(5.5));
            CHECK(rayhit.normal[0] == Approx(1.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{0.f, 7.5f, 0.f};
            Vec3r<T> dir1{0.f, -1.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK(hit_cylinder);
            CHECK(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == Approx(7.5));
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(1.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
        }
        SUBCASE("no hits") {
          SUBCASE("about z-axis") {
            const Vec3r<T> axis{0, 0, 1};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{0.f, 0.f, 7.5f};
            Vec3r<T> dir1{0.f, 0.f, 1.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK_FALSE(hit_cylinder);
            CHECK_FALSE(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{7.5f, 0.f, 0.f};
            Vec3r<T> dir1{1.f, 0.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK_FALSE(hit_cylinder);
            CHECK_FALSE(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{0.f, 7.5f, 0.f};
            Vec3r<T> dir1{0.f, 1.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK_FALSE(hit_cylinder);
            CHECK_FALSE(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
        }
      }
    }
    SUBCASE("shifted center") {
      centers->emplace_back(Vec3r<T>{1.f, 2.f, 0.f});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(2);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        SUBCASE("hits") {
          SUBCASE("origin above") {
            SUBCASE("perpendicular incidence on top") {

              Vec3r<T> org1{1.f, 2.f, 7.5f};
              Vec3r<T> dir1{0.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(5.5));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(1.f));
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{6.f, 2.f, 7.f};
              Vec3r<T> dir1{-1.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(50)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(1.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, 5};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, 5};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(-1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, 4};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, 4};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(-1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin below") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{1.f, 2.f, -7.5f};
              Vec3r<T> dir1{0.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(7.5));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(-1.f));
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{6.f, 2.f, -5.f};
              Vec3r<T> dir1{-1.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(50)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(-1.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, -3};
              Vec3r<T> dir1{-1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, -3};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(32)));
              CHECK(rayhit.normal[0] == Approx(-1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, -2};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, -2};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(std::sqrt(18)));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(-1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin around shell") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{6, 2, 1};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(4));
                CHECK(rayhit.normal[0] == Approx(1.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-4, 2, 1};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(4));
                CHECK(rayhit.normal[0] == Approx(-1.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{1, 7, 1};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(3));
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(1.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{1, -3, 1};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK(hit_cylinder);
                CHECK(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == Approx(3));
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(-1.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
            }
          }
          SUBCASE("origin inside of cylinder") {
            SUBCASE("hit top") {
              Vec3r<T> org1{1, 2, 1};
              Vec3r<T> dir1{0, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(1.f));
            }
            SUBCASE("hit bottom") {
              Vec3r<T> org1{1, 2, 1};
              Vec3r<T> dir1{0, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(-1.f));
            }
            SUBCASE("hit shell 1") {
              Vec3r<T> org1{1, 2, 1};
              Vec3r<T> dir1{-1, 0, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(-1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("hit shell 2") {
              Vec3r<T> org1{1, 2, 1};
              Vec3r<T> dir1{1, 0, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(1));
              CHECK(rayhit.normal[0] == Approx(1.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("hit shell 3") {
              Vec3r<T> org1{1, 2, 1};
              Vec3r<T> dir1{0, -1, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(2));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(-1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("hit shell 4") {
              Vec3r<T> org1{1, 2, 1};
              Vec3r<T> dir1{0, 1, 0};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK(hit_cylinder);
              CHECK(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == Approx(2));
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(1.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
        }
        SUBCASE("no hits") {
          SUBCASE("origin above (direction inverted)") {
            SUBCASE("perpendicular incidence on top") {

              Vec3r<T> org1{1.f, 2.f, 7.5f};
              Vec3r<T> dir1{0.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{6.f, 2.f, 7.f};
              Vec3r<T> dir1{1.f, 0.f, 1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, 5};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, 5};
              Vec3r<T> dir1{-1, 0, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, 4};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, 4};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin below (direction inverted)") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{1.f, 2.f, -7.5f};
              Vec3r<T> dir1{0.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{6.f, 2.f, -5.f};
              Vec3r<T> dir1{1.f, 0.f, -1.f};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, -3};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, -3};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, -2};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, -2};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;

              BVHTraceOptions<T> trace_options;

              CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              // Test intersections
              update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
              prepare_traversal(cylinder_intersector, ray, trace_options);
              T hit_distance = cylinder_intersector.hit_distance;
              const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
              update_intersector(cylinder_intersector, hit_distance, 0);
              post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

              CHECK_FALSE(hit_cylinder);
              CHECK_FALSE(rayhit.prim_id == 0);
              CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
              CHECK(rayhit.normal[0] == Approx(0.f));
              CHECK(rayhit.normal[1] == Approx(0.f));
              CHECK(rayhit.normal[2] == Approx(0.f));
            }
          }
          SUBCASE("origin around shell (direction inverted)") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{6, 2, 1};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-4, 2, 1};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{1, 7, 1};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{1, -3, 1};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                RayHit<T> rayhit;

                BVHTraceOptions<T> trace_options;

                CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                // Test intersections
                update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
                prepare_traversal(cylinder_intersector, ray, trace_options);
                T hit_distance = cylinder_intersector.hit_distance;
                const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
                update_intersector(cylinder_intersector, hit_distance, 0);
                post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

                CHECK_FALSE(hit_cylinder);
                CHECK_FALSE(rayhit.prim_id == 0);
                CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
                CHECK(rayhit.normal[0] == Approx(0.f));
                CHECK(rayhit.normal[1] == Approx(0.f));
                CHECK(rayhit.normal[2] == Approx(0.f));
              }
            }
          }
        }
      }
      SUBCASE("rotated") {
        SUBCASE("hits") {
          SUBCASE("about z-axis") {
            const Vec3r<T> axis{0, 0, 1};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{1.f, 2.f, 7.5f};
            Vec3r<T> dir1{0.f, 0.f, -1.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK(hit_cylinder);
            CHECK(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == Approx(5.5));
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(1.f));
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{8.5f, 2.f, 0.f};
            Vec3r<T> dir1{-1.f, 0.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK(hit_cylinder);
            CHECK(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == Approx(5.5));
            CHECK(rayhit.normal[0] == Approx(1.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{1.f, 9.5f, 0.f};
            Vec3r<T> dir1{0.f, -1.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK(hit_cylinder);
            CHECK(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == Approx(7.5));
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(1.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
        }
        SUBCASE("no hits") {
          SUBCASE("about z-axis") {
            const Vec3r<T> axis{0, 0, 1};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{1.f, 2.f, 7.5f};
            Vec3r<T> dir1{0.f, 0.f, 1.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK_FALSE(hit_cylinder);
            CHECK_FALSE(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{8.5f, 2.f, 0.f};
            Vec3r<T> dir1{1.f, 0.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK_FALSE(hit_cylinder);
            CHECK_FALSE(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{1.f, 5.5f, 0.f};
            Vec3r<T> dir1{0.f, 1.f, 0.f};

            Ray<T> ray{org1, dir1};
            RayHit<T> rayhit;

            BVHTraceOptions<T> trace_options;

            CylinderIntersector<T> cylinder_intersector{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            // Test intersections
            update_intersector(cylinder_intersector, ray.max_hit_distance, -1);
            prepare_traversal(cylinder_intersector, ray, trace_options);
            T hit_distance = cylinder_intersector.hit_distance;
            const bool hit_cylinder = intersect(cylinder_intersector, hit_distance, 0);
            update_intersector(cylinder_intersector, hit_distance, 0);
            post_traversal(cylinder_intersector, ray, hit_cylinder, rayhit);

            CHECK_FALSE(hit_cylinder);
            CHECK_FALSE(rayhit.prim_id == 0);
            CHECK(rayhit.hit_distance == std::numeric_limits<T>::max());
            CHECK(rayhit.normal[0] == Approx(0.f));
            CHECK(rayhit.normal[1] == Approx(0.f));
            CHECK(rayhit.normal[2] == Approx(0.f));
          }
        }
      }
    }
  }
}