//
// Created by raph on 05.05.20.
//

#include "../doctest.h"
#include "../test_helpers.h"
#include <blazert/datatypes.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("IntersectRayAABB", T, float, double)
{
    SUBCASE("template_example")
    {
    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 1.5000000000000002;
    T tmax_cor = 2.519803902718557;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ 1.0, 2.0, 2.0 };
    Vec3r<T> ray_org{ 0.0, -0.4708710135363803, 1.794174202707276 };
    Vec3r<T> ray_inv_dir{ pos_inf, 1.019803902718557, -5.099019513592786 };
    Vec3ui ray_dir_sign{ 0, 0, 1 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }
    
    SUBCASE("BB_is_point")
    {
    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 1.7320508075688772;
    T tmax_cor = 1.7320508075688772;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    T pos_inf = std::numeric_limits<T>().max();
    T neg_inf = -1 * std::numeric_limits<T>().max();
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ -1.0, 1.0, 1.0 };
    Vec3r<T> ray_org{ 0.0, 0.0, 0.0 };
    Vec3r<T> ray_inv_dir{ -1.732050807568877, 1.732050807568877, 1.732050807568877 };
    Vec3ui ray_dir_sign{ 1, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }
    
    SUBCASE("BB_flat_infinte")
    {
    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 1.7320508075688772;
    T tmax_cor = 1.7320508075688772;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ pos_inf, pos_inf, pos_inf };
    Vec3r<T> ray_org{ 0.0, 0.0, 0.0 };
    Vec3r<T> ray_inv_dir{ -1.732050807568877, 1.732050807568877, 1.732050807568877 };
    Vec3ui ray_dir_sign{ 1, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }        
    
    SUBCASE("BB_infinte")
    {
    // return variables    
    T tmin;
    T tmax;    
            
    // parameters
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();
    T min_t = 0.0;
    T max_t = pos_inf;

    // correct values
    T tmin_cor = 2.5;
    T tmax_cor = pos_inf;
        
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ pos_inf, pos_inf, pos_inf };
    Vec3r<T> ray_org{ -3.2613350843332274, 0.24622163855559087, 0.24622163855559087 };
    Vec3r<T> ray_inv_dir{ 1.1055415967851332, 3.3166247903554, 3.3166247903554 };
    Vec3ui ray_dir_sign{ 0, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }
        
}