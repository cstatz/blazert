#pragma once
#ifndef BLAZERT_DATATYPES_H_
#define BLAZERT_DATATYPES_H_

#include <blaze/Math.h>
#include <blaze/util/AlignedAllocator.h>

using blaze::StaticVector;
using blaze::DynamicVector;
using blaze::columnVector;
using blaze::aligned;
using blaze::unaligned;
using blaze::padded;
using blaze::unpadded;

// You can redefine these types according to you needs by by including the definitions prior to any blazert header and defining BLAZERT_DATATYPES_H_

namespace blazert {

// Vectors
template<class T> using Vec3r = StaticVector<T, 3UL, columnVector, aligned, padded>;
template<class T> using Vec2r = StaticVector<T, 2UL, columnVector, aligned, padded>;
using Vec3ui = StaticVector<unsigned int, 3UL, columnVector, aligned, padded>;
using vec2ui = StaticVector<unsigned int, 2UL, columnVector, aligned, padded>;

// Matrices
template<class T> using Mat3r = blaze::StaticMatrix<T, 3UL, 3UL, blaze::rowMajor, blaze::aligned, blaze::padded>;

// Container
template<class T> using Vec3rList = std::vector<Vec3r<T>, blaze::AlignedAllocator<Vec3r<T>>>;
using Vec3iList = std::vector<Vec3ui,  blaze::AlignedAllocator<Vec3ui>>;
template<typename T> using Mat3rList = std::vector<Mat3r<T>, blaze::AlignedAllocator<Mat3r<T>>>;
}
#endif  // BLAZERT_DATATYPES_H_
