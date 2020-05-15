//
// Created by ogarten on 1/29/19.
//

#include "doctest.h"
#include "test_helpers.h"
#include <blazert/datatypes.h>

using namespace blazert;


//TEST_CASE("Storage" * doctest::may_fail(true))
//{
//  {
//    Vec3rList<double> vertices;
//    Vec3iList indices;
//    const Vec3r<double> center{0, 0, 0};
//
//    cube_mesh_ccw(center, vertices, indices);
//
//    REQUIRE(is_aligned(&vertices[0], 16));
//    REQUIRE(is_aligned(&vertices[1], 16));
//
//    REQUIRE(is_aligned(&indices[0], 16));
//    REQUIRE(is_aligned(&indices[1], 16));
//  }
//  {
//    Vec3rList<float> vertices;
//    Vec3iList indices;
//    const Vec3r<float> center{0, 0, 0};
//
//    cube_mesh_ccw(center, vertices, indices);
//
//    REQUIRE(is_aligned(&vertices[0], 16));
//    REQUIRE(is_aligned(&vertices[1], 16));
//
//    REQUIRE(is_aligned(&indices[0], 16));
//    REQUIRE(is_aligned(&indices[1], 16));
//
//  }
//}

