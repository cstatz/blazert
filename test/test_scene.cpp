//
// Created by ogarten on 13/05/2020.
//

#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/sphere.h>
#include <blazert/ray.h>
#include <blazert/scene.h>
#include <memory>

#include "catch.hpp"

using namespace blazert;

TEMPLATE_TEST_CASE("Scene with Sphere", "[]", float, double) {
  SECTION("Intersection") {
    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto radiuss = std::make_unique<std::vector<TestType>>();

    centers->emplace_back(Vec3r<TestType>{0.});
    radiuss->emplace_back(1.);

    Vec3r<TestType> org{2.f, 0.f, 0.f};
    Vec3r<TestType> dir{-1.f, 0.f, 0.f};

    Scene<TestType> scene;
    unsigned int prim_id = scene.add_spheres(*centers, *radiuss);
    scene.commit();

    const Ray<TestType> ray{org, dir};
    RayHit<TestType> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    REQUIRE(prim_id == 0);
    REQUIRE(hit);
    REQUIRE(rayhit.hit_distance == Approx(1));
  }
}