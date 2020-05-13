//
// Created by ogarten on 12/05/2020.
//

#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/sphere.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../catch.hpp"
#include "../test_helpers.h"

using namespace blazert;

TEMPLATE_TEST_CASE("Sphere", "[bounding box, distance to surface, intersections]", float, double) {

  SECTION("bounding box") {
    const Vec3r<TestType> center{1.f, 1.f, 1.f};
    TestType radius = 1.f;

    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto radii = std::make_unique<std::vector<TestType>>();
    centers->emplace_back(center);
    radii->emplace_back(radius);

    Sphere<TestType> sphere(*centers, *radii);

    Vec3r<TestType> bmin, bmax;
    sphere.BoundingBox(bmin, bmax, 0);

    REQUIRE(bmin[0] == Approx(0.f));
    REQUIRE(bmin[1] == Approx(0.f));
    REQUIRE(bmin[2] == Approx(0.f));
    REQUIRE(bmax[0] == Approx(2.f));
    REQUIRE(bmax[1] == Approx(2.f));
    REQUIRE(bmax[2] == Approx(2.f));
  }
  SECTION("distance to surface") {
    Vec3r<TestType> center{0.f, 0.f, 0.f};
    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto radii = std::make_unique<std::vector<TestType>>();
    centers->emplace_back(center);
    
    SECTION("R = 1")
    {
      float radius = 1.f;
      radii->emplace_back(radius);
      
      Sphere<TestType> spheres(*centers, *radii);

      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{  0.f,  0.f,  0.f }, 0) == Approx(1.f));
      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{ -1.f,  0.f,  0.f }, 0) == Approx(0.f));
      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{ -2.f,  0.f,  0.f }, 0) == Approx(1.f));
      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{ -2.f, -2.f, -2.f }, 0) == Approx(std::sqrt(3.f * 4.f) - 1.f));
    }
    SECTION("R = 3")
    {
      float radius = 3.f;
      radii->emplace_back(radius);

      Sphere<TestType> spheres(*centers, *radii);

      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{  0.f,  0.f,  0.f }, 0) == Approx(3.f));
      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{ -1.f,  0.f,  0.f }, 0) == Approx(2.f));
      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{ -2.f,  0.f,  0.f }, 0) == Approx(1.f));
      REQUIRE(distance_to_surface(spheres, Vec3r<TestType>{ -2.f, -2.f, -2.f }, 0) == Approx(std::sqrt(3.f * 4.f) - 3.f));
    }
  }
  SECTION("intersections") {
    SECTION("Ray origin outside sphere") {
      const Vec3r<TestType> center{0.f, 0.f, 0.f};
      const TestType radius = 1.;

      // Centers and Radii should go on the heap.
      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center);
      radii->emplace_back(radius);

      const Vec3r<TestType> org{2.f, 0.f, 0.f};
      const Vec3r<TestType> dir{-1.f, 0.f, 0.f};

      const Ray<TestType> ray{org, dir};
      RayHit<TestType> rayhit;

      BVHTraceOptions<TestType> trace_options;

      SphereIntersector<TestType> sphere_intersector{*centers, *radii};

      // Test intersections
      update_intersector(sphere_intersector, ray.max_hit_distance, -1);
      prepare_traversal(sphere_intersector, ray, trace_options);
      TestType hit_distance = sphere_intersector.hit_distance;
      const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
      update_intersector(sphere_intersector, hit_distance, 0);
      post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

      // should be in distance of 1
      REQUIRE(hit_sphere);
      REQUIRE(rayhit.hit_distance == Approx(static_cast<TestType>(1.f)));
    }
    SECTION("Ray origin inside sphere") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      TestType radius = 1.f;

      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SECTION("Ray 1") {
        const Vec3r<TestType> org{0.f, 0.f, 0.f};
        const Vec3r<TestType> dir{-1.f, 0.f, 0.f};

        const Ray<TestType> ray{org, dir};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        SphereIntersector<TestType> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        TestType hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        // should be in distance of 1
        REQUIRE(hit_sphere);
        REQUIRE(rayhit.hit_distance == Approx(1.f));
      }
      SECTION("Ray 2") {
        const Vec3r<TestType> org{0.f, 0.f, 0.5f};
        const Vec3r<TestType> dir{0.f, 0.f, -1.f};

        const Ray<TestType> ray{org, dir};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        SphereIntersector<TestType> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        TestType hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        // should be in distance of 1
        REQUIRE(hit_sphere);
        REQUIRE(rayhit.hit_distance == Approx(1.5f));
      }
    }
    SECTION("Ray origin on sphere") {
      Vec3r<TestType> center1{1.f, 0.f, 0.f};
      TestType radius = 1.f;

      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SECTION("Ray 1") {
        Vec3r<TestType> org{0.f, 0.f, 0.f};
        Vec3r<TestType> dir{1.f, 0.f, 0.f};

        const Ray<TestType> ray{org, dir};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        SphereIntersector<TestType> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        TestType hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        REQUIRE(rayhit.hit_distance == Approx(2.f));
      }
      SECTION("Ray 2") {
        Vec3r<TestType> org{0.f, 0.f, 0.f};
        Vec3r<TestType> dir{-1.f, 0.f, 0.f};

        const Ray<TestType> ray{org, dir};
        RayHit<TestType> rayhit;

        BVHTraceOptions<TestType> trace_options;

        SphereIntersector<TestType> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        TestType hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        REQUIRE(rayhit.hit_distance == Approx(std::numeric_limits<TestType>::max()));
      }
    }
    SECTION("ray passing through sphere center") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      TestType radius = 2.f;

      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      Vec3r<TestType> org{3.f, 1.f, 0.f};
      Vec3r<TestType> dir{-1.f, 0.f, 0.f};

      const Ray<TestType> ray{org, dir};
      RayHit<TestType> rayhit;

      BVHTraceOptions<TestType> trace_options;

      SphereIntersector<TestType> sphere_intersector{*centers, *radii};

      // Test intersections
      update_intersector(sphere_intersector, ray.max_hit_distance, -1);
      prepare_traversal(sphere_intersector, ray, trace_options);
      TestType hit_distance = sphere_intersector.hit_distance;
      const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
      update_intersector(sphere_intersector, hit_distance, 0);
      post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

      REQUIRE(rayhit.hit_distance == Approx(1.26794919f));

      REQUIRE(rayhit.normal[0] == Approx(sqrt(3) / 2));
      REQUIRE(rayhit.normal[1] == Approx(sqrt(1) / 2));
      REQUIRE(rayhit.normal[2] == Approx(0.f));
    }
  }
}

