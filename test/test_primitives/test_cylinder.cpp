/*
 * Created by ogarten on 15/05/2020.
 * Modified on 11/06/202 by ogarten
 *
 * This file holds test cases for the cylinder primitive for the following cases:
 * 1. correct bounding box
 * 1.1 center at origin ( rotated, non-rotated cylinder)
 * 1.1 center not at origin (rotated, non-rotated cylinder)
 *
 * 2. testing primitive center function
 * 2.1 center at origin ( rotated, non-rotated cylinder)
 * 2.2 center not at origin ( rotated, non-rotated cylinder)
 *
 * 3. intersection tests
 * 3.1 center at origin
 *  3.1.1 non-rotated cylinder
 *      origin    hit where?
 *      __________________________
 *      above     top, perpendicular
 *      above     top, oblique indcidence
 *      above     shell surface
 *      below     bottom, perpendicular
 *      below     bottom, oblique indcidence
 *      below     shell surface
 *      outside   shell surface
 *      inside    top, bottom, shell surface
 *      above     no hits
 *      below     no hits
 *      outside   no hits
 *  3.1.2 rotated cylinder
 *      rotation  origin  hit where?
 *      ____________________________
 *      z-axis    z-axis  top + no hit
 *      y-axis    x-axis  top + no hit
 *      x-axis    y-axis  top + no hit
 *
 *
 * 3.2 center not at origin
 *  The same cases as in 3.1 are implemented for shifted center
 */

#define DOCTEST_CONFIG_INCLUDE_TYPE_TRAITS

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/cylinder.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../test_helpers.h"
#include "assert_helper.h"
#include <third_party/doctest/doctest/doctest.h>

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
      centers->emplace_back(Vec3r<T>{0, 0, 0});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(1);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_bmin{-1, -1, -1};
        const Vec3r<T> true_bmax{1, 1, 1};
        assert_bounding_box(cylinders, 0, true_bmin, true_bmax);
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_bmin{-1, -1, -1};
        const Vec3r<T> true_bmax{1, 1, 1};
        assert_bounding_box(cylinders, 0, true_bmin, true_bmax);
      }
    }
    SUBCASE("shifted center") {
      centers->emplace_back(Vec3r<T>{0, 1, 4});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(1);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_bmin{-1, 0, 3};
        const Vec3r<T> true_bmax{1, 2, 5};
        assert_bounding_box(cylinders, 0, true_bmin, true_bmax);
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_bmin{-1, 0, 3};
        const Vec3r<T> true_bmax{1, 2, 5};
        assert_bounding_box(cylinders, 0, true_bmin, true_bmax);
      }
    }
  }
  SUBCASE("primitive center") {
    SUBCASE("center at origin") {
      centers->emplace_back(Vec3r<T>{0, 0, 0});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(1);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_center{0, 0, 0};
        assert_primitive_center(cylinders, 0, true_center);
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_center{0, 0, 0};
        assert_primitive_center(cylinders, 0, true_center);
      }
    }
    SUBCASE("shifted center") {
      centers->emplace_back(Vec3r<T>{0, 1, 4});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(1);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_center{0, 1, 4};
        assert_primitive_center(cylinders, 0, true_center);
      }
      SUBCASE("rotated about (0,1,0)") {
        const Vec3r<T> axis{0, 1, 0};
        Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
        rotations->push_back(rot);
        CylinderCollection<T> cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);

        const Vec3r<T> true_center{0, 1, 4};
        assert_primitive_center(cylinders, 0, true_center);
      }
    }
  }
  SUBCASE("intersections") {
    SUBCASE("center at origin") {
      centers->emplace_back(Vec3r<T>{0, 0, 0});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(2);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        SUBCASE("hits") {
          SUBCASE("origin above") {
            SUBCASE("perpendicular incidence on top") {
              Vec3r<T> org1{0, 0, 7.5};
              Vec3r<T> dir1{0, 0, -1};

              Ray<T> ray{org1, dir1};

              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 6.5;
              const Vec3r<T> true_normal{0, 0, 1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{4, 0, 6};
              Vec3r<T> dir1{-1, 0, -1};

              Ray<T> ray{org1, dir1};

              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(50));
              const Vec3r<T> true_normal{0, 0, 1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{5, 0, 4};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, 4};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{-1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, 3};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, 1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, 3};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, -1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("origin below") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{0, 0, -8.5};
              Vec3r<T> dir1{0, 0, 1};

              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 7.5;
              const Vec3r<T> true_normal{0, 0, -1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{5, 0, -6};
              Vec3r<T> dir1{-1, 0, 1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(50));
              const Vec3r<T> true_normal{0, 0, -1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              const Vec3r<T> org1{5, 0, -4};
              const Vec3r<T> dir1{-1, 0, 1};
              const Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};
              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, -4};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{-1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, -3};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              RayHit<T> rayhit;
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, 1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, -3};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};
              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, -1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }

          SUBCASE("origin around shell") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{5, 0, 0};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 4;
                const Vec3r<T> true_normal{1, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-5, 0, 0};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 4;
                const Vec3r<T> true_normal{-1, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{0, 5, 0};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 3;
                const Vec3r<T> true_normal{0, 1, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{0, -5, 0};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 3;
                const Vec3r<T> true_normal{0, -1, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
            }
          }
          SUBCASE("origin inside of cylinder") {
            SUBCASE("hit top") {
              Vec3r<T> org1{0, 0, 0};
              Vec3r<T> dir1{0, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{0, 0, 1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit bottom") {
              Vec3r<T> org1{0, 0, 0};
              Vec3r<T> dir1{0, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{0, 0, -1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 1") {
              Vec3r<T> org1{0, 0, 0};
              Vec3r<T> dir1{-1, 0, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{-1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 2") {
              Vec3r<T> org1{0, 0, 0};
              Vec3r<T> dir1{1, 0, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 3") {
              Vec3r<T> org1{0, 0, 0};
              Vec3r<T> dir1{0, -1, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 2;
              const Vec3r<T> true_normal{0, -1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 4") {
              Vec3r<T> org1{0, 0, 0};
              Vec3r<T> dir1{0, 1, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 2;
              const Vec3r<T> true_normal{0, 1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
        }

        SUBCASE("no hits") {
          SUBCASE("origin above (direction inverted)") {
            SUBCASE("perpendicular incidence on top") {

              Vec3r<T> org1{0, 0, 6.5};
              Vec3r<T> dir1{0, 0, 1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{5, 0, 6};
              Vec3r<T> dir1{1, 0, 1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{5, 0, 4};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, 4};
              Vec3r<T> dir1{-1, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, 3};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, 3};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("origin below (direction inverted)") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{0, 0, -8.5};
              Vec3r<T> dir1{0, 0, -1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{5, 0, -6};
              Vec3r<T> dir1{1, 0, -1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{5, 0, -4};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-5, 0, -4};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{0, 5, -3};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{0, -5, -3};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("origin around shell (direction inverted") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{5, 0, 0};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-5, 0, 0};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{0, 5, 0};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{0, -5, 0};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
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

            Vec3r<T> org1{0, 0, 6.5};
            Vec3r<T> dir1{0, 0, -1};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = true;
            const unsigned int true_prim_id = 0;
            const T true_distance = 5.5;
            const Vec3r<T> true_normal{0, 0, 1};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{6.5, 0, 0};
            Vec3r<T> dir1{-1, 0, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = true;
            const unsigned int true_prim_id = 0;
            const T true_distance = 5.5;
            const Vec3r<T> true_normal{1, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{0, 8.5, 0};
            Vec3r<T> dir1{0, -1, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = true;
            const unsigned int true_prim_id = 0;
            const T true_distance = 7.5;
            const Vec3r<T> true_normal{0, 1, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
        }
        SUBCASE("no hits") {
          SUBCASE("about z-axis") {
            const Vec3r<T> axis{0, 0, 1};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{0, 0, 6.5};
            Vec3r<T> dir1{0, 0, 1};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = false;
            const unsigned int true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{6.5, 0, 0};
            Vec3r<T> dir1{1, 0, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = false;
            const unsigned int true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{0, 6.5, 0};
            Vec3r<T> dir1{0, 1, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = false;
            const unsigned int true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
        }
      }
    }

    SUBCASE("shifted center") {
      centers->emplace_back(Vec3r<T>{1, 2, 0});
      semi_axes_a->emplace_back(1);
      semi_axes_b->emplace_back(2);
      heights->emplace_back(2);
      SUBCASE("non-rotated") {
        rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));
        SUBCASE("hits") {
          SUBCASE("origin above") {
            SUBCASE("perpendicular incidence on top") {

              Vec3r<T> org1{1, 2, 6.5};
              Vec3r<T> dir1{0, 0, -1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 5.5;
              const Vec3r<T> true_normal{0, 0, 1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{6, 2, 6};
              Vec3r<T> dir1{-1, 0, -1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(50));
              const Vec3r<T> true_normal{0, 0, 1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, 4};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, 4};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{-1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, 3};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, 1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, 3};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, -1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("origin below") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{1, 2, -8.5};
              Vec3r<T> dir1{0, 0, 1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 7.5;
              const Vec3r<T> true_normal{0, 0, -1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{6, 2, -6};
              Vec3r<T> dir1{-1, 0, 1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(50));
              const Vec3r<T> true_normal{0, 0, -1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, -4};
              Vec3r<T> dir1{-1, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, -4};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(32));
              const Vec3r<T> true_normal{-1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, -3};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, 1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, -3};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = static_cast<T>(std::sqrt(18));
              const Vec3r<T> true_normal{0, -1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("origin around shell") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{6, 2, 0};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 4;
                const Vec3r<T> true_normal{1, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-4, 2, 0};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 4;
                const Vec3r<T> true_normal{-1, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{1, 7, 0};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 3;
                const Vec3r<T> true_normal{0, 1, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{1, -3, 0};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 3;
                const Vec3r<T> true_normal{0, -1, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
            }
          }
          SUBCASE("origin inside of cylinder") {
            SUBCASE("hit top") {
              Vec3r<T> org1{1, 2, 0};
              Vec3r<T> dir1{0, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{0, 0, 1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit bottom") {
              Vec3r<T> org1{1, 2, 0};
              Vec3r<T> dir1{0, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{0, 0, -1};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 1") {
              Vec3r<T> org1{1, 2, 0};
              Vec3r<T> dir1{-1, 0, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{-1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 2") {
              Vec3r<T> org1{1, 2, 0};
              Vec3r<T> dir1{1, 0, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 1;
              const Vec3r<T> true_normal{1, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 3") {
              Vec3r<T> org1{1, 2, 0};
              Vec3r<T> dir1{0, -1, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 2;
              const Vec3r<T> true_normal{0, -1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("hit shell 4") {
              Vec3r<T> org1{1, 2, 0};
              Vec3r<T> dir1{0, 1, 0};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 2;
              const Vec3r<T> true_normal{0, 1, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
        }
        SUBCASE("no hits") {
          SUBCASE("origin above (direction inverted)") {
            SUBCASE("perpendicular incidence on top") {

              Vec3r<T> org1{1, 2, 6.5};
              Vec3r<T> dir1{0, 0, 1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{6, 2, 6};
              Vec3r<T> dir1{1, 0, 1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, 4};
              Vec3r<T> dir1{1, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, 4};
              Vec3r<T> dir1{-1, 0, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, 3};
              Vec3r<T> dir1{0, 1, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, 3};
              Vec3r<T> dir1{0, -1, 1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("origin below (direction inverted)") {
            SUBCASE("perpendicular incidence on bottom") {
              Vec3r<T> org1{1, 2, -8.5};
              Vec3r<T> dir1{0, 0, -1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on bottom") {
              Vec3r<T> org1{6, 2, -6};
              Vec3r<T> dir1{1, 0, -1};

              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 1") {
              Vec3r<T> org1{6, 2, -4};
              Vec3r<T> dir1{1, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 2") {
              Vec3r<T> org1{-4, 2, -4};
              Vec3r<T> dir1{-1, 0, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 3") {
              Vec3r<T> org1{1, 7, -3};
              Vec3r<T> dir1{0, 1, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on shell 4") {
              Vec3r<T> org1{1, -3, -3};
              Vec3r<T> dir1{0, -1, -1};
              Ray<T> ray{org1, dir1};
              CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

              const bool true_hit = false;
              const unsigned int true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};
              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("origin around shell (direction inverted)") {
            SUBCASE("perpendicular incidence") {
              SUBCASE("origin: x+") {
                Vec3r<T> org1{6, 2, 0};
                Vec3r<T> dir1{1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: x-") {
                Vec3r<T> org1{-4, 2, 0};
                Vec3r<T> dir1{-1, 0, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y+") {
                Vec3r<T> org1{1, 7, 0};
                Vec3r<T> dir1{0, 1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("origin: y-") {
                Vec3r<T> org1{1, -3, 0};
                Vec3r<T> dir1{0, -1, 0};
                Ray<T> ray{org1, dir1};
                CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

                const bool true_hit = false;
                const unsigned int true_prim_id = static_cast<unsigned int>(-1);
                const T true_distance = std::numeric_limits<T>::max();
                const Vec3r<T> true_normal{0, 0, 0};
                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
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

            Vec3r<T> org1{1, 2, 6.5};
            Vec3r<T> dir1{0, 0, -1};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = true;
            const unsigned int true_prim_id = 0;
            const T true_distance = 5.5;
            const Vec3r<T> true_normal{0, 0, 1};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{7.5, 2, 0};
            Vec3r<T> dir1{-1, 0, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = true;
            const unsigned int true_prim_id = 0;
            const T true_distance = 5.5;
            const Vec3r<T> true_normal{1, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{1, 10.5, 0};
            Vec3r<T> dir1{0, -1, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = true;
            const unsigned int true_prim_id = 0;
            const T true_distance = 7.5;
            const Vec3r<T> true_normal{0, 1, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
        }
        SUBCASE("no hits") {
          SUBCASE("about z-axis") {
            const Vec3r<T> axis{0, 0, 1};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{1, 2, 7.5};
            Vec3r<T> dir1{0, 0, 1};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = false;
            const unsigned int true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about y-axis") {
            const Vec3r<T> axis{0, 1, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{8.5, 2, 0};
            Vec3r<T> dir1{1, 0, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = false;
            const unsigned int true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("about x-axis") {
            const Vec3r<T> axis{1, 0, 0};
            Mat3r<T> rot = arbitraryRotationMatrix(axis, pi<T> / 2);
            rotations->push_back(rot);

            Vec3r<T> org1{1, 5.5, 0};
            Vec3r<T> dir1{0, 1, 0};

            Ray<T> ray{org1, dir1};
            CylinderCollection<T> cylinders{*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations};

            const bool true_hit = false;
            const unsigned int true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(cylinders, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
        }
      }
    }
  }
}
