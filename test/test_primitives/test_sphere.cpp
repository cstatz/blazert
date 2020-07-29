//
// Created by ogarten on 12/05/2020.
//

#define DOCTEST_CONFIG_INCLUDE_TYPE_TRAITS

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/sphere.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../test_helpers.h"
#include "assert_helper.h"
#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("Sphere", T, float, double) {

  SUBCASE("bounding box") {
    const Vec3r<T> center{1, 1, 1};
    T radius = 1;

    auto centers = std::make_unique<Vec3rList<T>>();
    auto radii = std::make_unique<std::vector<T>>();
    centers->emplace_back(center);
    radii->emplace_back(radius);

    SphereCollection<T> spheres(*centers, *radii);

    const Vec3r<T> true_bmin{0, 0, 0};
    const Vec3r<T> true_bmax{2, 2, 2};
    assert_bounding_box(spheres, 0, true_bmin, true_bmax);
  }
  SUBCASE("distance to surface") {
    Vec3r<T> center{0, 0, 0};
    auto centers = std::make_unique<Vec3rList<T>>();
    auto radii = std::make_unique<std::vector<T>>();
    centers->emplace_back(center);

    SUBCASE("R = 1") {
      float radius = 1;
      radii->emplace_back(radius);

      SphereCollection<T> spheres(*centers, *radii);
      const unsigned int prim_id = 0;
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{0, 0, 0}, static_cast<T>(1.));
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{-1, 0, 0}, static_cast<T>(0));
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{-2, 0, 0}, static_cast<T>(1));
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{-2, -2, -2}, static_cast<T>(std::sqrt(3 * 4) - 1));
    }
    SUBCASE("R = 3") {
      float radius = 3;
      radii->emplace_back(radius);

      SphereCollection<T> spheres(*centers, *radii);
      const unsigned int prim_id = 0;
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{0, 0, 0}, static_cast<T>(3));
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{-1, 0, 0}, static_cast<T>(2));
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{-2, 0, 0}, static_cast<T>(1));
      assert_distance_to_surface(spheres, prim_id, Vec3r<T>{-2, -2, -2}, static_cast<T>(std::sqrt(3 * 4) - 3));
    }
  }
  SUBCASE("intersections") {
    SUBCASE("Ray origin outside sphere") {
      const Vec3r<T> center{0, 0, 0};
      const T radius = 1.;

      // Centers and Radii should go on the heap.
      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center);
      radii->emplace_back(radius);

      const Vec3r<T> org{2, 0, 0};
      const Vec3r<T> dir{-1, 0, 0};

      const Ray<T> ray{org, dir};

      SphereCollection spheres(*centers, *radii);

      const bool true_hit = true;
      const unsigned int true_prim_id = 0;
      const T true_distance = 1;
      const Vec3r<T> true_normal{1, 0, 0};

      SUBCASE("intersect primitive") {
        assert_intersect_primitive_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
      }
      SUBCASE("traverse bvh") {
        assert_traverse_bvh_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
      }
    }
    SUBCASE("Ray origin inside sphere") {
      Vec3r<T> center1{0, 0, 0};
      T radius = 1;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SUBCASE("Ray 1") {
        const Vec3r<T> org{0, 0, 0};
        const Vec3r<T> dir{-1, 0, 0};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 1;
        const Vec3r<T> true_normal{-1, 0, 0};

        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("Ray 2") {
        const Vec3r<T> org{0, 0, 0.5};
        const Vec3r<T> dir{0, 0, -1};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 1.5;
        const Vec3r<T> true_normal{0, 0, -1};

        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
    }

    SUBCASE("Ray origin on sphere") {
      Vec3r<T> center1{1, 0, 0};
      T radius = 1;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SUBCASE("Ray 1") {
        Vec3r<T> org{0, 0, 0};
        Vec3r<T> dir{1, 0, 0};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 2;
        const Vec3r<T> true_normal{1, 0, 0};

        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("traverse bvh") {
          assert_traverse_bvh_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
      SUBCASE("Ray 2") {
        Vec3r<T> org{0, 0, 0};
        Vec3r<T> dir{-1, 0, 0};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        const bool true_hit = false;
        const unsigned int true_prim_id = static_cast<unsigned int>(-1);
        const T true_distance = std::numeric_limits<T>::max();
        const Vec3r<T> true_normal{0, 0, 0};

        SUBCASE("intersect primitive") {
          assert_intersect_primitive_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
        SUBCASE("intersect primitive") {
          assert_traverse_bvh_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
        }
      }
    }

    SUBCASE("ray passing through sphere center") {
      Vec3r<T> center1{0, 0, 0};
      T radius = 2;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      Vec3r<T> org{3, 1, 0};
      Vec3r<T> dir{-1, 0, 0};

      const Ray<T> ray{org, dir};
      SphereCollection spheres(*centers, *radii);

      const bool true_hit = true;
      const unsigned int true_prim_id = 0;
      const T true_distance = static_cast<T>(1.26794919);
      const Vec3r<T> true_normal{std::sqrt(static_cast<T>(3)) / static_cast<T>(2.),
                                 std::sqrt(static_cast<T>(1)) / static_cast<T>(2.), static_cast<T>(0)};

      SUBCASE("intersect primitive") {
        assert_intersect_primitive_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
      }
      SUBCASE("traverse bvh") {
        assert_traverse_bvh_hit(spheres, ray, true_hit, true_prim_id, true_distance, true_normal);
      }
    }
  }
}