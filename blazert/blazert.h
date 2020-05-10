#pragma once
#ifndef BLAZERT_H
#define BLAZERT_H

#include <blazert/datatypes.h>
#include <blazert/defines.h>
#include <blazert/ray.h>

#ifdef EMBREE_TRACING
#include <blazert/embree/scene.h>
namespace blazert {
template<typename T>
using Scene = EmbreeScene<T>;
}
#else
#include <blazert/scene.h>
namespace blazert {
template<typename T>
using Scene = blazert::BlazertScene<T>;
}
#endif

#endif//BLAZERT_H
