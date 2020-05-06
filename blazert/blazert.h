#pragma once
#ifndef BLAZERT_H
#define BLAZERT_H

#include <blazert/datatypes.h>

#ifndef EMBREE_TRACING
#include <blazert/defines.h>
#include <blazert/ray.h>
#include <blazert/scene.h>
#else
#include <blazert/embree/scene.h>
#include <blazert/embree/ray.h>
#endif

#endif//BLAZERT_H
