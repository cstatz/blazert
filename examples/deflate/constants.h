//
// Created by Christoph Statz on 26.11.16.
//

#pragma once
#ifndef EM_ISOCONSTANTS_H
#define EM_ISOCONSTANTS_H

namespace emrt::constants {

template<typename T> constexpr T one_over_six =  T{1.}/T{6.};
template<typename T> constexpr T pi = 			 T{3.141592653589793238462643383279502884197};
template<typename T> constexpr T two_pi = 		 T{6.283185307179586476925286766559005768394};
template<typename T> constexpr T inv_two_pi = 		 T{1.0 / two_pi<T>};
template<typename T> constexpr T four_pi = 		 T{12.56637061435917295385057353311801153679};
template<typename T> constexpr T half_pi = 		 T{1.570796326794896619231321691639751442099};
template<typename T> constexpr T inv_half_pi =		 T{0.6366197723675813430755350534900574};
template<typename T> constexpr T inv_sqrt_four_pi = 	 T{0.2820947917738781434740397257803862929220};
template<typename T> constexpr T euler = 		 T{2.718281828459045235360287471352662498};
template<typename T> constexpr T ln2 = 			 T{0.6931471805599453094172321214581766};
template<typename T> constexpr T inv_ln2 = 		 T{1.4426950408889634073599246810018921};
template<typename T> constexpr T ln10 = 		 T{2.3025850929940456840179914546843642};
template<typename T> constexpr T deg2rad =		 T{pi<T> / 180.0};
template<typename T> constexpr T rad2deg = 		 T{180.0 / pi<T>};
template<typename T> constexpr T vacuum_speed_of_light = T{299792458.};
template<typename T> constexpr T vacuum_permittivity = 	 T{8.854187817620389850536563031710750260608e-12};
template<typename T> constexpr T vacuum_permeability = 	 T{four_pi<T> * 1.0e-7};
template<typename T> constexpr T vacuum_waveimpedance =  T{376.7303134617706554681984004203193082686};

} // namespace em::constants

#endif // EM_ISOCONSTANTS_H
