//
// Created by tobiask on 22.09.20.
//

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

TEST_CASE_TEMPLATE("Trimesh", T, float, double) {
  auto centers = std::make_unique<Vec3rList<T>>();
  auto vertices = std::make_unique<Vec3rList<T>>();
  auto indices = std::make_unique<Vec3iList>();
  SUBCASE("single triangle cw") {
    SUBCASE("bounding box") {
      SUBCASE("center at origin") {
        const Vec3r<T> center{0, 0, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        const Vec3r<T> true_bmin{-1, -1, -1};
        const Vec3r<T> true_bmax{1, 1, 1};

        assert_bounding_box(triangles_cw, 0, true_bmin, true_bmax);
      }

      SUBCASE("shifted center") {
        const Vec3r<T> center{0, 1, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        const Vec3r<T> true_bmin{-1, 0, -1};
        const Vec3r<T> true_bmax{1, 2, 1};

        assert_bounding_box(triangles_cw, 0, true_bmin, true_bmax);
      }
    }

    SUBCASE("primitive center") {
      SUBCASE("center at origin") {
        const Vec3r<T> center{0, 0, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        // focus triangle  = 1/3 * (v1 + v2 + v3)
        const Vec3r<T> calc_focus_triangle{0.33333, 0.33333, -0.33333};
        assert_primitive_center(triangles_cw, 0, calc_focus_triangle);
      }

      SUBCASE("shifted center") {
        const Vec3r<T> center{1, 0, 0};

        single_triangle_cw(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);

        // focus triangle  = 1/3 * (v1 + v2 + v3)
        const Vec3r<T> calc_focus_triangle{1.33333f, 0.33333f, -0.33333f};
        assert_primitive_center(triangles_cw, 0, calc_focus_triangle);
      }
    }

    SUBCASE("intersections") {
      SUBCASE("center at origin") {
        const Vec3r<T> center{0, 0, 0};
        single_triangle_cw_flat_xy(center, *vertices, *indices);
        TriangleMesh triangles_cw(*vertices, *indices);
        SUBCASE("hits"){
          SUBCASE("origin above"){
            Vec3r<T> org1{1, 1, 1};
            Vec3r<T> dir1{-1,-1,-1};

            Ray<T> ray{org1, dir1};

            const bool true_hit = true;
            const unsigned int true_prim_id = 0;
//            const T true_distance =

          }
//          SUBCASE("origin below")
        }


      }
    }
  }
}
