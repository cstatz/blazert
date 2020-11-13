/*
 * Created by tobiask on 22.09.20.
 *
 *
 * This file holds test cases for the trimesh primitive for the following cases:
 * 1. single triangle cw
 *  1.1 correct bounding box
 *    1.1.1 center at origin
 *    1.1.2 center shifted
 *
 *  1.2 testing primitive center function
 *    1.2.1 center at origin
 *    1.2.2 center shifted
 *
 *  1.3 intersection tests
 *    1.3.1 center at origin
 *        origin    hit where?
 *        __________________________
 *        above       top
 *
 * 2. cube mesh cw
 *  2.1 correct bounding box
 */

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/trimesh.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../test_helpers.h"
#include "assert_helper.h"
#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("Trimesh : Single Triangle", T, float, double) {
  auto centers = std::make_unique<Vec3rList<T>>();
  auto vertices = std::make_unique<Vec3rList<T>>();
  auto indices = std::make_unique<Vec3iList>();
  SUBCASE("1. single triangle cw") {
    SUBCASE("1.1 bounding box") {
      SUBCASE("1.1.1 center at origin (triangle across the room)") {
        const Vec3r<T> center{0, 0, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        const Vec3r<T> true_bmin{-1, -1, -1};
        const Vec3r<T> true_bmax{1, 1, 1};

        assert_bounding_box(triangles_cw, 0, true_bmin, true_bmax);
      }
      SUBCASE("1.1.1 center at origin (triangle in xy plane)") {
        const Vec3r<T> center{0, 0, 0};

        single_triangle_cw_flat_xy(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        const Vec3r<T> true_bmin{0, 0, 0};
        const Vec3r<T> true_bmax{1, 1, 0};

        assert_bounding_box(triangles_cw, 0, true_bmin, true_bmax);
      }
      SUBCASE("1.1.2 shifted center") {
        const Vec3r<T> center{0, 1, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        const Vec3r<T> true_bmin{-1, 0, -1};
        const Vec3r<T> true_bmax{1, 2, 1};

        assert_bounding_box(triangles_cw, 0, true_bmin, true_bmax);
      }
    }

    SUBCASE("1.2 primitive center") {
      SUBCASE("center at origin") {
        const Vec3r<T> center{0, 0, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        // focus triangle  = 1/3 * (v1 + v2 + v3)
        const Vec3r<T> calc_focus_triangle{static_cast<T>(0.33333), static_cast<T>(0.33333), static_cast<T>(-0.33333)};
        assert_primitive_center(triangles_cw, 0, calc_focus_triangle);
      }

      SUBCASE("shifted center") {
        const Vec3r<T> center{1, 0, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        // focus triangle  = 1/3 * (v1 + v2 + v3)
        const Vec3r<T> calc_focus_triangle{static_cast<T>(1.33333), static_cast<T>(0.33333), static_cast<T>(-0.33333)};
        assert_primitive_center(triangles_cw, 0, calc_focus_triangle);
      }
    }

    SUBCASE("1.3 intersections") {
      SUBCASE("1.3.1 center at origin") {
        const Vec3r<T> center{0, 0, 0};

        SUBCASE("hits") {
          SUBCASE("assembled cw  /  flat xz") {
            single_triangle_cw_flat_xz(center, *vertices, *indices);
            TriangleMesh triangles(*vertices, *indices);
            SUBCASE("right") {
              Vec3r<T> org1{0.25, 5, 0.25};
              Vec3r<T> dir1{0, -1, 0};

              Ray<T> ray{org1, dir1};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 5;
              const Vec3r<T> true_normal{0, 1, 0};// depending on the direction in which the triangles are assembled

              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
          SUBCASE("assembled cw  /  flat yz") {
            single_triangle_cw_flat_yz(center, *vertices, *indices);
            TriangleMesh triangles(*vertices, *indices);
            SUBCASE("right") {
              Vec3r<T> org1{5, 0.25, 0.25};
              Vec3r<T> dir1{-1, 0, 0};

              Ray<T> ray{org1, dir1};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 5;
              const Vec3r<T> true_normal{-1, 0, 0};// depending on the direction in which the triangles are assembled

              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }

          SUBCASE("assembled cw / flat xy") {
            single_triangle_cw_flat_xy(center, *vertices, *indices);
            TriangleMesh triangles(*vertices, *indices);
            SUBCASE("origin above") {
              SUBCASE("perpendicular incidence on top") {
                Vec3r<T> org1{0.25, 0.25, 5};
                Vec3r<T> dir1{0, 0, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 5;
                const Vec3r<T> true_normal{0, 0, -1};// depending on the direction in which the triangles are assembled

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("oblique incidence on top") {
                Vec3r<T> org1{1, 1, 1};
                Vec3r<T> dir1{-0.75, -0.75, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(std::sqrt(1 + 2 * 0.75 * 0.75));
                const Vec3r<T> true_normal{0, 0, -1};// depending on the direction in which the triangles are assembled

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("edge x axis") {
                Vec3r<T> org1{0.5, 0, 5};
                Vec3r<T> dir1{0, 0, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("edge y axis") {
                Vec3r<T> org1{0, 0.5, 5};
                Vec3r<T> dir1{0, 0, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("edge hypotenuse") {
                Vec3r<T> org1{0.5, 0.5, 5};
                Vec3r<T> dir1{0, 0, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("corner center") {
                Vec3r<T> org1{0, 0, 5};
                Vec3r<T> dir1{0, 0, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("corner y axis") {
                Vec3r<T> org1{0, 1, 5};
                Vec3r<T> dir1{0, 0, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("corner x axis") {
                Vec3r<T> org1{1, 0, 5};
                Vec3r<T> dir1{0, 0, -1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
            }
            SUBCASE("origin below") {
              SUBCASE("perpendicular incidence from below") {
                Vec3r<T> org1{0.25, 0.25, -5};
                Vec3r<T> dir1{0, 0, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = 5;
                const Vec3r<T> true_normal{0, 0, -1};// depending on the direction in which the triangles are assembled

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("oblique incidenc from below") {
                Vec3r<T> org1{1, 1, -1};
                Vec3r<T> dir1{-0.75, -0.75, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(std::sqrt(1 + 2 * 0.75 * 0.75));
                const Vec3r<T> true_normal{0, 0, -1};// depending on the direction in which the triangles are assembled

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("edge 1") {
                Vec3r<T> org1{0.5, 0, -5};
                Vec3r<T> dir1{0, 0, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("edge 2") {
                Vec3r<T> org1{0, 0.5, -5};
                Vec3r<T> dir1{0, 0, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("edge 3") {
                Vec3r<T> org1{0.5, 0.5, -5};
                Vec3r<T> dir1{0, 0, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("corner 1") {
                Vec3r<T> org1{0, 0, -5};
                Vec3r<T> dir1{0, 0, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("corner 2") {
                Vec3r<T> org1{0, 1, -5};
                Vec3r<T> dir1{0, 0, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
              SUBCASE("corner 3") {
                Vec3r<T> org1{1, 0, -5};
                Vec3r<T> dir1{0, 0, 1};

                Ray<T> ray{org1, dir1};

                const bool true_hit = true;
                const unsigned int true_prim_id = 0;
                const T true_distance = static_cast<T>(5.0);
                const Vec3r<T> true_normal{0, 0, -1};// dep

                SUBCASE("intersect primitive") {
                  assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
                SUBCASE("traverse bvh") {
                  assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
                }
              }
            }
          }
        }
        SUBCASE("no hits") {
          single_triangle_cw_flat_xy(center, *vertices, *indices);
          TriangleMesh triangles(*vertices, *indices);
          SUBCASE("origin above (inverted direction)") {
            SUBCASE("perpendicular incidence on top") {
              Vec3r<T> org1{0.25, 0.25, 5};
              Vec3r<T> dir1{0, 0, 1};

              Ray<T> ray{org1, dir1};

              const bool true_hit = false;
              auto true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};// depending on the direction in which the triangles are assembled

              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
            SUBCASE("oblique incidence on top") {
              Vec3r<T> org1{1, 1, 1};
              Vec3r<T> dir1{-0.75, -0.75, 1};

              Ray<T> ray{org1, dir1};

              const bool true_hit = false;
              auto true_prim_id = static_cast<unsigned int>(-1);
              const T true_distance = std::numeric_limits<T>::max();
              const Vec3r<T> true_normal{0, 0, 0};// depending on the direction in which the triangles are assembled

              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
        }

        SUBCASE("assembled ccw") {
          single_triangle_ccw_flat_xy(center, *vertices, *indices);
          TriangleMesh triangles(*vertices, *indices);

          SUBCASE("hits") {
            SUBCASE("origin above") {
              Vec3r<T> org1{0.25, 0.25, 5};
              Vec3r<T> dir1{0, 0, -1};

              Ray<T> ray{org1, dir1};

              const bool true_hit = true;
              const unsigned int true_prim_id = 0;
              const T true_distance = 5;
              const Vec3r<T> true_normal{0, 0, 1};// depending on the direction in which the triangles are assembled

              SUBCASE("intersect primitive") {
                assert_intersect_primitive_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
              SUBCASE("traverse bvh") {
                assert_traverse_bvh_hit(triangles, ray, true_hit, true_prim_id, true_distance, true_normal);
              }
            }
          }
        }
      }
    }
  }
}

TEST_CASE_TEMPLATE("Trimesh: Cube Mesh - Bounding Box", T, float, double) {
  auto centers = std::make_unique<Vec3rList<T>>();
  auto vertices = std::make_unique<Vec3rList<T>>();
  auto indices = std::make_unique<Vec3iList>();
  SUBCASE("2.1.1 center at origin") {
    const Vec3r<T> center{0, 0, 0};
    SUBCASE("counter clockwise") {

      cube_mesh_ccw(center, *vertices, *indices);
      TriangleMesh triangles_ccw(*vertices, *indices);

      const Vec3r<T> true_bmin{-1, -1, -1};
      const Vec3r<T> true_bmax{1, 1, 1};

      assert_bounding_box_multi_prim_id(triangles_ccw, 0, true_bmin, true_bmax, 8);
    }
    SUBCASE("clockwise") {
      cube_mesh_cw(center, *vertices, *indices);
      TriangleMesh triangles_cw(*vertices, *indices);

      const Vec3r<T> true_bmin{-1, -1, -1};
      const Vec3r<T> true_bmax{1, 1, 1};

      assert_bounding_box_multi_prim_id(triangles_cw, 0, true_bmin, true_bmax, 8);
    }
  }
  SUBCASE("2.1.2 center shifted") {
    const Vec3r<T> center{4, 2, 0};
    SUBCASE("counter clockwise") {
      cube_mesh_ccw_01(center, *vertices, *indices);
      TriangleMesh triangles_ccw(*vertices, *indices);

      const Vec3r<T> true_bmin{3, 1, -1};
      const Vec3r<T> true_bmax{5, 3, 1};

      assert_bounding_box_multi_prim_id(triangles_ccw, 0, true_bmin, true_bmax, 12);
    }
    SUBCASE("clockwise") {
      cube_mesh_cw(center, *vertices, *indices);
      TriangleMesh triangles_cw(*vertices, *indices);

      const Vec3r<T> true_bmin{3, 1, -1};
      const Vec3r<T> true_bmax{5, 3, 1};

      assert_bounding_box_multi_prim_id(triangles_cw, 0, true_bmin, true_bmax, 12);
    }
  }
}

// https://en.cppreference.com/w/cpp/types/numeric_limits
TEST_CASE_TEMPLATE("Trimesh : Cube Mesh - Intersections", T, float, double) {
  auto centers = std::make_unique<Vec3rList<T>>();
  auto vertices = std::make_unique<Vec3rList<T>>();
  auto indices = std::make_unique<Vec3iList>();
  SUBCASE("Center at origin") {
    const Vec3r<T> center{0, 0, 0};
    SUBCASE("assembled ccw") {
      cube_mesh_ccw_01(center, *vertices, *indices);
      TriangleMesh triangle_cw(*vertices, *indices);
      SUBCASE("source outside") {
        SUBCASE("HIT") {
          SUBCASE("positive Z-Axis") {
            Vec3r<T> org1{0, 0, 5};
            Vec3r<T> dir1{0, 0, -1};

            Ray<T> ray{org1, dir1};

            const bool true_hit = true;
            const T true_distance = 4;

            assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);

            //        const Vec3r<T> true_normal{0, 0, -1};
            //        assert_traverse_bvh_hit_trimesh(triangle_cw, ray, true_hit, true_distance, true_normal);
          }
          SUBCASE("negativ Z-Axis") {
            Vec3r<T> org1{0, 0, -5};
            Vec3r<T> dir1{0, 0, 1};

            Ray<T> ray{org1, dir1};

            const bool true_hit = true;
            const T true_distance = 4;

            assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
          }
          SUBCASE("positive Y-Axis") {
            Vec3r<T> org1{0, 5, 0};
            Vec3r<T> dir1{0, -1, 0};

            Ray<T> ray{org1, dir1};

            const bool true_hit = true;
            const T true_distance = 4;

            assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
          }
          SUBCASE("negativ Y-Axis") {
            Vec3r<T> org1{0, -5, 0};
            Vec3r<T> dir1{0, 1, 0};

            Ray<T> ray{org1, dir1};

            const bool true_hit = true;
            const T true_distance = 4;

            assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
          }
          SUBCASE("positive X-Axis") {
            Vec3r<T> org1{5, 0, 0};
            Vec3r<T> dir1{-1, 0, 0};

            Ray<T> ray{org1, dir1};

            const bool true_hit = true;
            const T true_distance = 4;

            assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
          }
          SUBCASE("negativ X-Axis") {
            Vec3r<T> org1{-5, 0, 0};
            Vec3r<T> dir1{1, 0, 0};

            Ray<T> ray{org1, dir1};

            const bool true_hit = true;
            const T true_distance = 4;

            assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
          }
        }
        SUBCASE("NO HIT") {
          SUBCASE("ray parallel to z-Axis") {
            Vec3r<T> org1{2, 2, -5};
            Vec3r<T> dir1{0, 0, 1};

            Ray<T> ray{org1, dir1};

            const bool true_hit = false;
            auto true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(triangle_cw, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(triangle_cw, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("ray parallel to y-Axis") {
            Vec3r<T> org1{2, -5, 2};
            Vec3r<T> dir1{0, 1, 0};

            Ray<T> ray{org1, dir1};

            const bool true_hit = false;
            auto true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(triangle_cw, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(triangle_cw, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("ray parallel to x-Axis") {
            Vec3r<T> org1{-5, 2, 2};
            Vec3r<T> dir1{1, 0, 0};

            Ray<T> ray{org1, dir1};

            const bool true_hit = false;
            auto true_prim_id = static_cast<unsigned int>(-1);
            const T true_distance = std::numeric_limits<T>::max();
            const Vec3r<T> true_normal{0, 0, 0};
            SUBCASE("intersect primitive") {
              assert_intersect_primitive_hit(triangle_cw, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
            SUBCASE("traverse bvh") {
              assert_traverse_bvh_hit(triangle_cw, ray, true_hit, true_prim_id, true_distance, true_normal);
            }
          }
          SUBCASE("source inside") {
            SUBCASE("source at cube origin center") {
              SUBCASE("Hit : center of planes") {
                SUBCASE("Ray-Direction: +Z") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, 0, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = 1;

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("Ray-Direction: -Z") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, 0, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = 1;

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("Ray-Direction: +Y") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, 1, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = 1;

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("Ray-Direction: -Y") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, -1, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = 1;

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("Ray-Direction: +X") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, 0, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = 1;

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("Ray-Direction: -X") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, 0, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = 1;

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
              }
              SUBCASE("Hit : edges") {
                SUBCASE("center of edge = (1, 1, 0)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, 1, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (1, -1, 0)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, -1, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (-1, 1, 0)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, 1, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (-1, -1, 0)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, -1, 0};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }

                SUBCASE("center of edge = (1, 0, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, 0, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (1, 0, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, 0, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (-1, 0, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, 0, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (-1, 0, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, 0, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }

                SUBCASE("center of edge = (0, 1, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, 1, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (0, 1, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, 1, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (0, -1, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, -1, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("center of edge = (0, -1, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{0, -1, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(2));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
              }
              SUBCASE("Hit : corners") {
                SUBCASE("corner  = (1, 1, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, 1, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("corner  = (1, -1, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, -1, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("corner  = (-1, 1, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, 1, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("corner  = (-1, -1, 1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, -1, 1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("corner  = (1, 1, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, 1, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("corner  = (1, -1, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{1, -1, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("corner  = (-1, 1, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, 1, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
                SUBCASE("corner  = (-1, -1, -1)") {
                  Vec3r<T> org1{0, 0, 0};
                  Vec3r<T> dir1{-1, -1, -1};

                  Ray<T> ray{org1, dir1};

                  const bool true_hit = true;
                  const T true_distance = static_cast<T>(std::sqrt(3));

                  assert_traverse_bvh_hit_trimesh_temp(triangle_cw, ray, true_hit, true_distance);
                }
              }
            }
          }
        }
      }
    }
  }
}

TEST_CASE_TEMPLATE("Trimesh : Pyramid - Intersections", T, float, double) {
  auto centers = std::make_unique<Vec3rList<T>>();
  auto vertices = std::make_unique<Vec3rList<T>>();
  auto indices = std::make_unique<Vec3iList>();
  SUBCASE("center at orgin") {
    const Vec3r<T> center{0, 0, 0};

    pyramid_mesh(center, *vertices, *indices);
    TriangleMesh triangle(*vertices, *indices);

    Vec3r<T> org1{0, 0, 5};
    Vec3r<T> dir1{0, 0, -1};

    Ray<T> ray{org1, dir1};

    const bool true_hit = true;
    const T true_distance = 4;
    const Vec3r<T> true_normal{0, 0, 1};

    assert_traverse_bvh_hit_trimesh(triangle, ray, true_hit, true_distance, true_normal);
  }
}
/* TEST_CASE_TEMPLATE("Trimesh : Two Triangles - Intersections", T, float, double) {
  auto centers = std::make_unique<Vec3rList<T>>();
  auto vertices = std::make_unique<Vec3rList<T>>();
  auto indices = std::make_unique<Vec3iList>();
  SUBCASE("center at origin") {
    const Vec3r<T> center{0, 0, 0};
    SUBCASE("xy plane") {
      two_triangle_ccw_plane_xy(center, *vertices, *indices);
      TriangleMesh triangle(*vertices, *indices);

      Vec3r<T> org1{0, 0, 5};
      Vec3r<T> dir1{0, 0, -1};

      Ray<T> ray{org1, dir1};

      const bool true_hit = true;
      const T true_distance = 5;
      const Vec3r<T> true_normal{0, 0, -1};

      assert_traverse_bvh_hit_trimesh(triangle, ray, true_hit, true_distance, true_normal);
    }
    SUBCASE("xz plane") {
      two_triangle_ccw_plane_xz(center, *vertices, *indices);
      TriangleMesh triangle(*vertices, *indices);

      Vec3r<T> org1{0, 5, 0};
      Vec3r<T> dir1{0, -1, 0};

      Ray<T> ray{org1, dir1};

      const bool true_hit = true;
      const T true_distance = 5;
      const Vec3r<T> true_normal{0, 1, 0};

      assert_traverse_bvh_hit_trimesh(triangle, ray, true_hit, true_distance, true_normal);
    }
    SUBCASE("two planes in xy") {
      two_plane_mesh(center, *vertices, *indices);
      TriangleMesh triangle(*vertices, *indices);

      Vec3r<T> org1{0, 0, 5};
      Vec3r<T> dir1{0, 0, -1};

      Ray<T> ray{org1, dir1};

      const bool true_hit = true;
      const T true_distance = 4;
      const Vec3r<T> true_normal{0, 0, -1};

      assert_traverse_bvh_hit_trimesh(triangle, ray, true_hit, true_distance, true_normal);
    }
    SUBCASE("two planes 90 degree") {
      two_plane_mesh_90_deg(center, *vertices, *indices);
      TriangleMesh triangle(*vertices, *indices);

      Vec3r<T> org1{0, 0, 5};
      Vec3r<T> dir1{0, 0, -1};

      Ray<T> ray{org1, dir1};

      const bool true_hit = true;
      const T true_distance = 4;
      const Vec3r<T> true_normal{0, 0, -1};

      assert_traverse_bvh_hit_trimesh(triangle, ray, true_hit, true_distance, true_normal);
    }
  }
  SUBCASE("center shifted") {
    const Vec3r<T> center{0, 0, -5};
    SUBCASE("two planes 90 degree") {
      two_plane_mesh_90_deg(center, *vertices, *indices);
      TriangleMesh triangle(*vertices, *indices);

      Vec3r<T> org1{0, 0, 0};
      Vec3r<T> dir1{0, 0, -1};

      Ray<T> ray{org1, dir1};

      const bool true_hit = true;
      const T true_distance = 4;
      const Vec3r<T> true_normal{0, 0, -1};

      assert_traverse_bvh_hit_trimesh(triangle, ray, true_hit, true_distance, true_normal);
    }
  }
}*/
