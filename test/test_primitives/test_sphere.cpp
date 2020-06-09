//
// Created by ogarten on 12/05/2020.
//

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
    const Vec3r<T> center{1.f, 1.f, 1.f};
    T radius = 1.f;

    auto centers = std::make_unique<Vec3rList<T>>();
    auto radii = std::make_unique<std::vector<T>>();
    centers->emplace_back(center);
    radii->emplace_back(radius);

    SphereCollection<T> spheres(*centers, *radii);

    const auto [bmin, bmax] = spheres.get_primitive_bounding_box(0);

    CHECK(bmin[0] == Approx(0.f));
    CHECK(bmin[1] == Approx(0.f));
    CHECK(bmin[2] == Approx(0.f));
    CHECK(bmax[0] == Approx(2.f));
    CHECK(bmax[1] == Approx(2.f));
    CHECK(bmax[2] == Approx(2.f));
  }
  SUBCASE("distance to surface") {
    Vec3r<T> center{0.f, 0.f, 0.f};
    auto centers = std::make_unique<Vec3rList<T>>();
    auto radii = std::make_unique<std::vector<T>>();
    centers->emplace_back(center);
    
    SUBCASE("R = 1")
    {
      float radius = 1.f;
      radii->emplace_back(radius);
      
      SphereCollection<T> spheres(*centers, *radii);

      CHECK(spheres.distance_to_surface(Vec3r<T>{  0.f,  0.f,  0.f }, 0) == Approx(1.f));
      CHECK(spheres.distance_to_surface(Vec3r<T>{ -1.f,  0.f,  0.f }, 0) == Approx(0.f));
      CHECK(spheres.distance_to_surface(Vec3r<T>{ -2.f,  0.f,  0.f }, 0) == Approx(1.f));
      CHECK(spheres.distance_to_surface(Vec3r<T>{ -2.f, -2.f, -2.f }, 0) == Approx(std::sqrt(3.f * 4.f) - 1.f));
    }
    SUBCASE("R = 3")
    {
      float radius = 3.f;
      radii->emplace_back(radius);

      SphereCollection<T> spheres(*centers, *radii);

      CHECK(spheres.distance_to_surface(Vec3r<T>{  0.f,  0.f,  0.f }, 0) == Approx(3.f));
      CHECK(spheres.distance_to_surface(Vec3r<T>{ -1.f,  0.f,  0.f }, 0) == Approx(2.f));
      CHECK(spheres.distance_to_surface(Vec3r<T>{ -2.f,  0.f,  0.f }, 0) == Approx(1.f));
      CHECK(spheres.distance_to_surface(Vec3r<T>{ -2.f, -2.f, -2.f }, 0) == Approx(std::sqrt(3.f * 4.f) - 3.f));
    }
  }
  SUBCASE("intersections") {
    SUBCASE("Ray origin outside sphere") {
      const Vec3r<T> center{0.f, 0.f, 0.f};
      const T radius = 1.;

      // Centers and Radii should go on the heap.
      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center);
      radii->emplace_back(radius);

      const Vec3r<T> org{2.f, 0.f, 0.f};
      const Vec3r<T> dir{-1.f, 0.f, 0.f};

      const Ray<T> ray{org, dir};

      SphereCollection spheres(*centers, *radii);

      const bool true_hit = true;
      const unsigned int true_prim_id = 0;
      const T true_distance = 1;
      const Vec3r<T> true_normal{1, 0, 0};

      test_intersect_primitive_hit(spheres, ray, true_prim_id, true_distance, true_normal);
      test_traverse_bvh_hit(spheres, ray, true_prim_id, true_distance, true_normal);
    }
    SUBCASE("Ray origin inside sphere") {
      Vec3r<T> center1{0.f, 0.f, 0.f};
      T radius = 1.f;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SUBCASE("Ray 1") {
        const Vec3r<T> org{0.f, 0.f, 0.f};
        const Vec3r<T> dir{-1.f, 0.f, 0.f};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        const unsigned int true_prim_id = 0;
        const T true_distance = 1;
        const Vec3r<T> true_normal{-1, 0, 0};

        test_intersect_primitive_hit(spheres, ray, true_prim_id, true_distance, true_normal);
        test_traverse_bvh_hit(spheres, ray, true_prim_id, true_distance, true_normal);
      }
      SUBCASE("Ray 2") {
        const Vec3r<T> org{0.f, 0.f, 0.5f};
        const Vec3r<T> dir{0.f, 0.f, -1.f};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        const unsigned int true_prim_id = 0;
        const T true_distance = 1.5;
        const Vec3r<T> true_normal{0, 0, -1};

        test_intersect_primitive_hit(spheres, ray, true_prim_id, true_distance, true_normal);
        test_traverse_bvh_hit(spheres, ray, true_prim_id, true_distance, true_normal);
      }
    }

    SUBCASE("Ray origin on sphere") {
      Vec3r<T> center1{1.f, 0.f, 0.f};
      T radius = 1.f;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SUBCASE("Ray 1") {
        Vec3r<T> org{0.f, 0.f, 0.f};
        Vec3r<T> dir{1.f, 0.f, 0.f};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        const bool true_hit = true;
        const unsigned int true_prim_id = 0;
        const T true_distance = 2;
        const Vec3r<T> true_normal{1, 0, 0};

        test_intersect_primitive_hit(spheres, ray, true_prim_id, true_distance, true_normal);
        test_traverse_bvh_hit(spheres, ray, true_prim_id, true_distance, true_normal);
      }
      SUBCASE("Ray 2") {
        Vec3r<T> org{0.f, 0.f, 0.f};
        Vec3r<T> dir{-1.f, 0.f, 0.f};

        const Ray<T> ray{org, dir};

        SphereCollection spheres(*centers, *radii);

        test_intersect_primitive_no_hit(spheres, ray);
        test_traverse_bvh_no_hit(spheres, ray);
      }
    }

    SUBCASE("ray passing through sphere center") {
      Vec3r<T> center1{0.f, 0.f, 0.f};
      T radius = 2.f;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      Vec3r<T> org{3.f, 1.f, 0.f};
      Vec3r<T> dir{-1.f, 0.f, 0.f};

      const Ray<T> ray{org, dir};
      SphereCollection spheres(*centers, *radii);

      const bool true_hit = true;
      const unsigned int true_prim_id = 0;
      const T true_distance = 1.26794919f;
      const Vec3r<T> true_normal{std::sqrt(static_cast<T>(3)) / static_cast<T>(2.),
                                 std::sqrt(static_cast<T>(1)) / static_cast<T>(2.),
                                 static_cast<T>(0)};

      test_intersect_primitive_hit(spheres, ray, true_prim_id, true_distance, true_normal);
      test_traverse_bvh_hit(spheres, ray, true_prim_id, true_distance, true_normal);
    }
  }
}