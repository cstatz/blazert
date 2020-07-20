#pragma once
#ifndef BLAZERT_H
#define BLAZERT_H

#include <blazert/datatypes.h>
#include <blazert/defines.h>
#include <blazert/ray.h>
#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>

#ifdef EMBREE_TRACING
#include <blazert/embree/scene.h>
namespace blazert {
template<typename T>
using Scene = EmbreeScene<T>;
}
#else
#include <blazert/scene.h>
namespace blazert {
template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
using Scene = blazert::BlazertScene<T, BVH_T, Builder>;
}
#endif

#endif//BLAZERT_H
