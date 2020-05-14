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

TEMPLATE_TEST_CASE("Scene with Plane", "[]", float, double) {
  SECTION("Intersection") {
    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto dxs = std::make_unique<std::vector<TestType>>();
    auto dys = std::make_unique<std::vector<TestType>>();
    auto rotations = std::make_unique<Mat3rList<TestType>>();

    centers->emplace_back(Vec3r<TestType>{0., 0., 0.});
    dxs->emplace_back(2.);
    dys->emplace_back(2.);
    rotations->emplace_back(blaze::IdentityMatrix<TestType>(3UL));

    Vec3r<TestType> org{0.f, 0.f, 5.f};
    Vec3r<TestType> dir{0.f, 0.f, -1.f};

    Scene<TestType> scene;
    unsigned int prim_id = scene.add_planes(*centers, *dxs, *dys, *rotations);
    scene.commit();

    const Ray<TestType> ray{org, dir};
    RayHit<TestType> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    REQUIRE(prim_id == 0);
    REQUIRE(hit);
    REQUIRE(rayhit.hit_distance == Approx(5));
    REQUIRE(rayhit.normal[0] == Approx(0));
    REQUIRE(rayhit.normal[1] == Approx(0));
    REQUIRE(rayhit.normal[2] == Approx(1));
  }
}