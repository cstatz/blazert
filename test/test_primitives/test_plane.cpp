/*
 * Created by ogarten on 13/05/2020.
 * Modified on 11/06/202 by ogarten
 *
 * This file holds test cases for the plane primitive for the following cases:
 * 1. correct bounding box
 * 1.1 center at origin ( rotated, non-rotated cylinder)
 * 1.1 center not at origin (rotated, non-rotated cylinder)
 *
 * 2. testing primitive center function
 * 2.1 center at origin ( rotated, non-rotated cylinder)
 * 2.2 center not at origin ( rotated, non-rotated cylinder)
 *
 * 3. testing distance to surface function
 * 3.1 center at origin ( rotated, non-rotated cylinder)
 * 3.2 center not at origin ( rotated, non-rotated cylinder)
 *
 * 4. intersection tests
 * 4.1 center at origin
 *  4.1.1 non-rotated plane
 *  4.1.2 rotated, hit on plane
 *  4.1.3 rotated, edge hit
 *  4.1.4 non-rotated, ray origin outside sphere bounds
 *  4.1.5 non-rotated, all edge hits
 *  4.1.6 non-rotated, all corner hits
 *
 * 4.2 center not at origin
 *  3.1.1 - 3.1.6 are implemented here as well
 */

#define DOCTEST_CONFIG_INCLUDE_TYPE_TRAITS

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/plane.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../test_helpers.h"
#include "assert_helper.h"
#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("Plane", T, float, double) {
  const T d1 = 2.;
  const T d2 = 2.;

  auto centers = std::make_unique<Vec3rList<T>>();
  auto dxx = std::make_unique<std::vector<T>>();
  auto dyy = std::make_unique<std::vector<T>>();
  auto rotations = std::make_unique<Mat3rList<T>>();

  dxx->push_back(d1);
  dyy->push_back(d2);

  SUBCASE("bounding box") {
    SUBCASE("center at origin") {
      const Vec3r<T> center{0., 0., 0.};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        const Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{-1., -1., -std::numeric_limits<T>::min()};
        const Vec3r<T> true_bmax{1., 1., std::numeric_limits<T>::min()};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }

      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{-std::numeric_limits<T>::min(), -1., -1.};
        const Vec3r<T> true_bmax{std::numeric_limits<T>::min(), 1., 1.};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {

        // plane on z=0, later rotated to x = 0
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<T> axis{static_cast<T>(1 / std::sqrt(2)), static_cast<T>(1 / std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{-1., -1., static_cast<T>(-std::sqrt(2.))};
        const Vec3r<T> true_bmax{1., 1., static_cast<T>(std::sqrt(2.))};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<T> axis{0, 0, 1};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 4);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{-static_cast<T>(std::sqrt(2.)), -static_cast<T>(std::sqrt(2.)), 0};
        const Vec3r<T> true_bmax{static_cast<T>(std::sqrt(2.)), static_cast<T>(std::sqrt(2.)), 0};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }
    }
    SUBCASE("shifted center") {
      const Vec3r<T> center{1., 1., 1.};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        const Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{0, 0, 1};
        const Vec3r<T> true_bmax{2, 2, 1};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }
      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{1, 0, 0};
        const Vec3r<T> true_bmax{1, 2, 2};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {
        // plane on z=0, rotated such that (xmin, ymin) corner is parallel to z-axis
        const Vec3r<T> axis{static_cast<T>(1 / std::sqrt(2)), static_cast<T>(1 / std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{0, 0, 1 - static_cast<T>(std::sqrt(2))};
        const Vec3r<T> true_bmax{2, 2, 1 + static_cast<T>(std::sqrt(2))};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<T> axis{0, 0, 1};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 4);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const Vec3r<T> true_bmin{1 - static_cast<T>(std::sqrt(2)), 1 - static_cast<T>(std::sqrt(2)), 1};
        const Vec3r<T> true_bmax{1 + static_cast<T>(std::sqrt(2)), 1 + static_cast<T>(std::sqrt(2)), 1};
        assert_bounding_box(planes, 0, true_bmin, true_bmax);
      }
    }
  }
  SUBCASE("primitive center") {
    SUBCASE("center at origin") {
      const Vec3r<T> center{0., 0., 0.};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        const Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }

      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {

        // plane on z=0, later rotated to x = 0
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<T> axis{static_cast<T>(1 / std::sqrt(2)), static_cast<T>(1 / std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<T> axis{0, 0, 1};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 4);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }
    }
    SUBCASE("shifted center") {
      const Vec3r<T> center{1., 1., 1.};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        const Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }
      SUBCASE("rotated about y-axis") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }
      SUBCASE("rotated about r=normalized(1,1,0)") {
        // plane on z=0, rotated such that (xmin, ymin) corner is parallel to z-axis
        const Vec3r<T> axis{static_cast<T>(1 / std::sqrt(2)), static_cast<T>(1 / std::sqrt(2)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }
      SUBCASE("rotated in xy-plane") {
        const Vec3r<T> axis{0, 0, 1};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 4);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        assert_primitive_center(planes, 0, center);
      }
    }
  }

  SUBCASE("distance to surface") {
    SUBCASE("centered") {
      const Vec3r<T> center{0.f, 0.f, 0.f};
      centers->emplace_back(center);
      SUBCASE("non-rotated") {
        const Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        const unsigned int prim_id = 0;
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{0.f, 0.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{0.f, 0.f, -1.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, 1.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, -1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, -1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, -1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, -1.f}, static_cast<T>(1.));
      }
      SUBCASE("rotated") {
        // x = 0 is now plane equation
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        const unsigned int prim_id = 0;
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 0.f, 0.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 0.f, 0.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, -1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, -1.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, -1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, 1.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, -1.f}, static_cast<T>(1.));
      }
    }
    SUBCASE("shifted") {
      const Vec3r<T> center{0.f, 0.f, 1.f};
      centers->emplace_back(center);
      SUBCASE("non-rotated") {
        const Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        const unsigned int prim_id = 0;
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{0.f, 0.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{0.f, 0.f, 0.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, 2.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, 0.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, 0.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, 0.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, 0.f}, static_cast<T>(1.));
      }
      SUBCASE("rotated") {
        // x = 0 is now plane equation
        Mat3r<T> rot{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        const unsigned int prim_id = 0;
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 0.f, 0.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 0.f, 0.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, 1.f, 0.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{1.f, -1.f, 0.f}, static_cast<T>(1.));

        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, 1.f, 0.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, 2.f}, static_cast<T>(1.));
        assert_distance_to_surface(planes, prim_id, Vec3r<T>{-1.f, -1.f, 0.f}, static_cast<T>(1.));
      }
    }
  }

  SUBCASE("intersections") {
    SUBCASE("center at origin") {
      const Vec3r<T> center{0., 0., 0.};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        const Vec3r<T> org1{0., 0., 5.};
        const Vec3r<T> dir1{0., 0., -1.};
        const Ray<T> ray{org1, dir1};

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 5;
        const Vec3r<T> true_normal{0, 0, 1};
        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("rotated about (0,1,0)") {
        // matrix which rotates the plane 45 degrees about the z-axis ( x = 0 is now
        // plane eq)
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);

        rotations->push_back(rot);

        Vec3r<T> org1{5., 0., 0.};
        Vec3r<T> dir1{-1., 0., 0.};
        const Ray<T> ray{org1, dir1};
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 5;
        const Vec3r<T> true_normal{1, 0, 0};
        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("rotated about normalized(1,1,0), edge hit") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        const Vec3r<T> axis{static_cast<T>(1. / std::sqrt(2.)), static_cast<T>(1. / std::sqrt(2.)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);

        const Vec3r<T> org1{-5, 5, 0.5};
        const Vec3r<T> dir1{1, -1, 0};
        const Ray<T> ray{org1, dir1};
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = static_cast<T>(5. * std::sqrt(2.));
        const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(2)), 1 / static_cast<T>(std::sqrt(2)), 0};
        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("non-rotated, ray origin outside plane bounds") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        SUBCASE("outside on positive x-axis") {
          Vec3r<T> org1{4., 0., 4.};
          Vec3r<T> dir1{-1., 0., -1.};
          const Ray<T> ray{org1, dir1};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = static_cast<T>(4 * std::sqrt(2));
          const Vec3r<T> true_normal{0, 0, 1};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("outside on negative x-axis") {
          Vec3r<T> org2{-4, 0, -4};
          Vec3r<T> dir2{1, 0, 1};
          const Ray<T> ray{org2, dir2};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = static_cast<T>(4. * std::sqrt(2.));
          const Vec3r<T> true_normal{0, 0, -1};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          //          CHECK(hit_plane);
          //          CHECK(rayhit2.prim_id == 0);
          //          CHECK(rayhit2.hit_distance == Approx(4. * std::sqrt(2.)));
          //          CHECK(rayhit2.normal[0] == Approx(0.));
          //          CHECK(rayhit2.normal[1] == Approx(0.));
          //          CHECK(rayhit2.normal[2] == Approx(-1.));
        }
      }
      SUBCASE("non-rotated, edge intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        SUBCASE("edge: x max") {
          // edge at x_max
          Vec3r<T> org1{1., 0., 4.};
          Vec3r<T> dir1{0., 0., -1.};
          const Ray<T> ray{org1, dir1};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{1 / static_cast<T>(std::sqrt(2)), 0, 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("edge: x_min") {
          // edge at x_min
          Vec3r<T> org2{-1., 0., 4.};
          Vec3r<T> dir2{0., 0., -1.};
          const Ray<T> ray{org2, dir2};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(2)), 0, 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }

          //          // hits from the negative x direction -> normal vector should point towards
          //          // that direction
          //          CHECK(hit_plane);
          //          CHECK(rayhit.prim_id == 0);
          //          CHECK(rayhit.hit_distance == Approx(4));
          //          CHECK(rayhit.normal[0] == Approx(-1. / std::sqrt(2)));
          //          CHECK(rayhit.normal[1] == Approx(0.));
          //          CHECK(rayhit.normal[2] == Approx(1. / std::sqrt(2)));
        }
        SUBCASE("edge: y max") {
          Vec3r<T> org3{0., 1., 4.};
          Vec3r<T> dir3{0., 0., -1.};
          const Ray<T> ray{org3, dir3};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{0, 1 / static_cast<T>(std::sqrt(2)), 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }

          // hits from the negative x direction -> normal vector should point towards
          //          // that direction
          //          CHECK(hit_plane);
          //          CHECK(rayhit.prim_id == 0);
          //          CHECK(rayhit.hit_distance == Approx(4));
          //          CHECK(rayhit.normal[0] == Approx(0.));
          //          CHECK(rayhit.normal[1] == Approx(1. / std::sqrt(2)));
          //          CHECK(rayhit.normal[2] == Approx(1. / std::sqrt(2)));
        }
        SUBCASE("edge: y min") {
          // edge at y_min
          Vec3r<T> org4{0., -1., 4.};
          Vec3r<T> dir4{0., 0., -1.};
          const Ray<T> ray{org4, dir4};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{0, -1 / static_cast<T>(std::sqrt(2)), 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }

          // hits from the negative x direction -> normal vector should point towards
          // that direction
          //          CHECK(hit_plane);
          //          CHECK(rayhit.prim_id == 0);
          //          CHECK(rayhit.hit_distance == Approx(4));
          //          CHECK(rayhit.normal[0] == Approx(0.));
          //          CHECK(rayhit.normal[1] == Approx(-1. / std::sqrt(2)));
          //          CHECK(rayhit.normal[2] == Approx(1. / std::sqrt(2)));
        }
      }
      SUBCASE("non-rotated, corner intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);
        SUBCASE("corner: x max, y max") {
          Vec3r<T> org1{1., 1., 4.};
          Vec3r<T> dir1{0., 0., -1.};
          const Ray<T> ray{org1, dir1};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{1 / static_cast<T>(std::sqrt(3)), 1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("corner: x min, y max") {
          Vec3r<T> org2{-1., 1., 4.};
          Vec3r<T> dir2{0., 0., -1.};
          const Ray<T> ray{org2, dir2};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(3)), 1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("corner: x max, y min") {
          Vec3r<T> org3{1., -1., 4.};
          Vec3r<T> dir3{0., 0., -1.};
          const Ray<T> ray{org3, dir3};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{1 / static_cast<T>(std::sqrt(3)), -1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("corner: x min, y min") {
          // corner at x_min, y_min
          Vec3r<T> org4{-1., -1., 4.};
          Vec3r<T> dir4{0., 0., -1.};
          const Ray<T> ray{org4, dir4};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4;
          const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(3)), -1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
      }
    }
    SUBCASE("shifted center") {
      const Vec3r<T> center{1., 1., 1.};
      centers->emplace_back(center);

      SUBCASE("non-rotated") {
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);

        const Vec3r<T> org1{1., 1., 5.};
        const Vec3r<T> dir1{0., 0., -1.};
        const Ray<T> ray{org1, dir1};
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 4;
        const Vec3r<T> true_normal{0, 0, 1};
        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("rotated, non-shifted") {
        // matrix which rotates the plane 90 degrees about the y-axis ( x = 0 is now
        // plane eq)
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);

        Vec3r<T> org1{5., 1., 1.};
        Vec3r<T> dir1{-1., 0., 0.};
        const Ray<T> ray{org1, dir1};
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 4;
        const Vec3r<T> true_normal{1, 0, 0};
        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("rotated about r=normalized(1,1,0), edge hit") {
        // matrix which rotates the plane about axis = normalized(1,1,0)
        const Vec3r<T> axis{static_cast<T>(1. / std::sqrt(2.)), static_cast<T>(1. / std::sqrt(2.)), 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);

        const Vec3r<T> org1{-4., 6., 0.5};
        const Vec3r<T> dir1{1., -1., 0.};
        const Ray<T> ray{org1, dir1};
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 5 * static_cast<T>(std::sqrt(2));
        const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(2)), 1 / static_cast<T>(std::sqrt(2)), 0};
        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("non-rotated, ray origin outside plane bounds") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        SUBCASE("outside on positive x-axis") {
          Vec3r<T> org1{5., 1., 5.};
          Vec3r<T> dir1{-1., 0., -1.};
          const Ray<T> ray{org1, dir1};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4 * static_cast<T>(std::sqrt(2));
          const Vec3r<T> true_normal{0, 0, 1};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("outside on negative x-axis") {
          Vec3r<T> org2{-3., 1., -3.};
          Vec3r<T> dir2{1., 0., 1.};
          const Ray<T> ray{org2, dir2};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 4 * static_cast<T>(std::sqrt(2));
          const Vec3r<T> true_normal{0, 0, -1};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
      }
      SUBCASE("non-rotated, edge intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        SUBCASE("edge: x max") {
          // edge at x_max
          Vec3r<T> org1{2., 1., 4.};
          Vec3r<T> dir1{0., 0., -1.};
          const Ray<T> ray{org1, dir1};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{1 / static_cast<T>(std::sqrt(2)), 0, 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("edge: x_min") {
          // edge at x_min
          Vec3r<T> org2{0., 1., 4.};
          Vec3r<T> dir2{0., 0., -1.};
          const Ray<T> ray{org2, dir2};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(2)), 0, 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("edge: y max") {
          Vec3r<T> org3{1., 2., 4.};
          Vec3r<T> dir3{0., 0., -1.};
          const Ray<T> ray{org3, dir3};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{0, 1 / static_cast<T>(std::sqrt(2)), 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("edge: y min") {
          // edge at y_min
          Vec3r<T> org4{1., 0., 4.};
          Vec3r<T> dir4{0., 0., -1.};
          const Ray<T> ray{org4, dir4};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{0, -1 / static_cast<T>(std::sqrt(2)), 1 / static_cast<T>(std::sqrt(2))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
      }
      SUBCASE("non-rotated, corner intersection") {
        // matrix which rotates the plane about the y-axis ( x = 0 is now plane eq)
        Mat3r<T> rot = blaze::IdentityMatrix<T>(3UL);
        rotations->push_back(rot);
        PlaneCollection<T> planes(*centers, *dxx, *dyy, *rotations);

        SUBCASE("corner: x max, y max") {
          Vec3r<T> org1{2., 2., 4.};
          Vec3r<T> dir1{0., 0., -1.};
          const Ray<T> ray{org1, dir1};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{1 / static_cast<T>(std::sqrt(3)), 1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("corner: x min, y max") {
          Vec3r<T> org2{0., 2., 4.};
          Vec3r<T> dir2{0., 0., -1.};
          const Ray<T> ray{org2, dir2};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(3)), 1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("corner: x max, y min") {
          Vec3r<T> org3{2., 0., 4.};
          Vec3r<T> dir3{0., 0., -1.};
          const Ray<T> ray{org3, dir3};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{1 / static_cast<T>(std::sqrt(3)), -1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
        SUBCASE("corner: x min, y min") {
          // corner at x_min, y_min
          Vec3r<T> org4{0., 0., 4.};
          Vec3r<T> dir4{0., 0., -1.};
          const Ray<T> ray{org4, dir4};

          const bool true_hit = true;
          const unsigned int true_prim_id = 0;
          const T true_distance = 3;
          const Vec3r<T> true_normal{-1 / static_cast<T>(std::sqrt(3)), -1 / static_cast<T>(std::sqrt(3)),
                                     1 / static_cast<T>(std::sqrt(3))};
          SUBCASE("intersect primitive") {
            assert_intersect_primitive_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
          SUBCASE("traverse bvh") {
            assert_traverse_bvh_hit(planes, ray, true_hit, true_prim_id, true_distance, true_normal);
          }
        }
      }
    }
  }
}
