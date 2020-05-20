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

#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("Scene with Sphere", T, float, double) {
  SUBCASE("Intersection") {
    auto centers = std::make_unique<Vec3rList<T>>();
    auto radiuss = std::make_unique<std::vector<T>>();

    centers->emplace_back(Vec3r<T>{0.});
    radiuss->emplace_back(1.);

    Vec3r<T> org{2.f, 0.f, 0.f};
    Vec3r<T> dir{-1.f, 0.f, 0.f};

    Scene<T> scene;
    unsigned int prim_id = scene.add_spheres(*centers, *radiuss);
    scene.commit();

    const Ray<T> ray{org, dir};
    RayHit<T> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    REQUIRE(prim_id == 0);
    REQUIRE(prim_id == rayhit.prim_id);
    REQUIRE(hit);
    REQUIRE(rayhit.hit_distance == Approx(1));
  }
}

TEST_CASE_TEMPLATE("Scene with Plane", T, float, double) {
  SUBCASE("Intersection") {
    auto centers = std::make_unique<Vec3rList<T>>();
    auto dxs = std::make_unique<std::vector<T>>();
    auto dys = std::make_unique<std::vector<T>>();
    auto rotations = std::make_unique<Mat3rList<T>>();

    centers->emplace_back(Vec3r<T>{0., 0., 0.});
    dxs->emplace_back(2.);
    dys->emplace_back(2.);
    rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));

    Vec3r<T> org{0.f, 0.f, 5.f};
    Vec3r<T> dir{0.f, 0.f, -1.f};

    Scene<T> scene;
    unsigned int prim_id = scene.add_planes(*centers, *dxs, *dys, *rotations);
    scene.commit();

    const Ray<T> ray{org, dir};
    RayHit<T> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    REQUIRE(prim_id == 0);
    REQUIRE(prim_id == rayhit.prim_id);
    REQUIRE(hit);
    REQUIRE(rayhit.hit_distance == Approx(5));
    REQUIRE(rayhit.normal[0] == Approx(0));
    REQUIRE(rayhit.normal[1] == Approx(0));
    REQUIRE(rayhit.normal[2] == Approx(1));
  }
}

TEST_CASE_TEMPLATE("Scene with Cylinder", T, float, double) {
  SUBCASE("Intersection") {
    auto centers = std::make_unique<Vec3rList<T>>();
    auto semi_axes_a = std::make_unique<std::vector<T>>();
    auto semi_axes_b = std::make_unique<std::vector<T>>();
    auto heights = std::make_unique<std::vector<T>>();
    auto rotations = std::make_unique<Mat3rList<T>>();

    centers->emplace_back(Vec3r<T>{0., 0., 0.});
    semi_axes_a->emplace_back(2.);
    semi_axes_b->emplace_back(2.);
    heights->emplace_back(2.);
    rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));

    Vec3r<T> org{0.f, 0.f, 5.f};
    Vec3r<T> dir{0.f, 0.f, -1.f};

    Scene<T> scene;
    unsigned int prim_id = scene.add_cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);
    scene.commit();

    const Ray<T> ray{org, dir};
    RayHit<T> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    REQUIRE(prim_id == 0);
    REQUIRE(prim_id == rayhit.prim_id);
    REQUIRE(hit);
    REQUIRE(rayhit.hit_distance == Approx(3));
    REQUIRE(rayhit.normal[0] == Approx(0));
    REQUIRE(rayhit.normal[1] == Approx(0));
    REQUIRE(rayhit.normal[2] == Approx(1));
  }
}