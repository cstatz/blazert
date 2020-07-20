//
// Created by ogarten on 13/05/2020.
//

#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
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

    centers->emplace_back(Vec3r<T>{0., 0., 0.});
    radiuss->emplace_back(1.);

    Vec3r<T> org{2.f, 0.f, 0.f};
    Vec3r<T> dir{-1.f, 0.f, 0.f};

    Scene<T, BVH, SAHBinnedBuilder> scene;
    unsigned int geom_id = scene.add_spheres(*centers, *radiuss);
    scene.commit();

    const Ray<T> ray{org, dir};
    RayHit<T> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    CHECK(geom_id == 0);
    CHECK(geom_id == rayhit.geom_id);
    CHECK(hit);
    CHECK(rayhit.hit_distance == Approx(1));
    CHECK(rayhit.normal[0] == Approx(1));
    CHECK(rayhit.normal[1] == Approx(0));
    CHECK(rayhit.normal[2] == Approx(0));
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

    Scene<T, BVH, SAHBinnedBuilder> scene;
    unsigned int geom_id = scene.add_planes(*centers, *dxs, *dys, *rotations);
    scene.commit();

    const Ray<T> ray{org, dir};
    RayHit<T> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    CHECK(geom_id == 0);
    CHECK(geom_id == rayhit.geom_id);
    CHECK(hit);
    CHECK(rayhit.hit_distance == Approx(5));
    CHECK(rayhit.normal[0] == Approx(0));
    CHECK(rayhit.normal[1] == Approx(0));
    CHECK(rayhit.normal[2] == Approx(1));
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

    Vec3r<T> org{0.f, 0.f, 4.f};
    Vec3r<T> dir{0.f, 0.f, -1.f};

    Scene<T, BVH, SAHBinnedBuilder> scene;
    unsigned int geom_id = scene.add_cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);
    scene.commit();

    const Ray<T> ray{org, dir};
    RayHit<T> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    CHECK(geom_id == 0);
    CHECK(geom_id == rayhit.geom_id);
    CHECK(hit);
    CHECK(rayhit.hit_distance == Approx(3));
    CHECK(rayhit.normal[0] == Approx(0));
    CHECK(rayhit.normal[1] == Approx(0));
    CHECK(rayhit.normal[2] == Approx(1));
  }
}

TEST_CASE_TEMPLATE("Scene with 2 different primitives", T, float, double) {
  SUBCASE("Intersection") {
    //Cylinder
    auto c_centers = std::make_unique<Vec3rList<T>>();
    auto c_semi_axes_a = std::make_unique<std::vector<T>>();
    auto c_semi_axes_b = std::make_unique<std::vector<T>>();
    auto c_heights = std::make_unique<std::vector<T>>();
    auto c_rotations = std::make_unique<Mat3rList<T>>();

    c_centers->emplace_back(Vec3r<T>{0., 0., -2.});
    c_semi_axes_a->emplace_back(2.);
    c_semi_axes_b->emplace_back(2.);
    c_heights->emplace_back(2.);
    c_rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));

    // Sphere
    auto s_centers = std::make_unique<Vec3rList<T>>();
    auto s_radii = std::make_unique<std::vector<T>>();

    s_centers->emplace_back(Vec3r<T>{0., 0., 3.});
    s_radii->emplace_back(2.);

    // ray
    const Vec3r<T> org{0.f, 0.f, 7.f};
    const Vec3r<T> dir{0.f, 0.f, -1.f};
    const Ray<T> ray{org, dir};

    // scene + add primitives -> cylinder with sphere above it, only sphere should be hit!
    Scene<T, BVH, SAHBinnedBuilder> scene;
    unsigned int c_geom_id = scene.add_cylinders(*c_centers, *c_semi_axes_a, *c_semi_axes_b, *c_heights, *c_rotations);
    unsigned int s_geom_id = scene.add_spheres(*s_centers, *s_radii);
    scene.commit();

    RayHit<T> rayhit;

    const bool hit = intersect1(scene, ray, rayhit);

    CHECK(c_geom_id == 0);
    CHECK(s_geom_id == 1);
    CHECK(hit);
    CHECK(rayhit.prim_id == 0);
    CHECK(rayhit.geom_id == s_geom_id);
    CHECK(rayhit.hit_distance == Approx(2));
    CHECK(rayhit.normal[0] == Approx(0));
    CHECK(rayhit.normal[1] == Approx(0));
    CHECK(rayhit.normal[2] == Approx(1));
  }
}
