#pragma once
#ifndef BLAZERT_DEFINES_H_
#define BLAZERT_DEFINES_H_

#ifndef BLAZERT_MAX_STACK_DEPTH
#define BLAZERT_MAX_STACK_DEPTH 256
#endif

#ifndef BLAZERT_MAX_TREE_DEPTH
#define BLAZERT_MAX_TREE_DEPTH 28
#endif

// Defines relevant for the parallel build
#ifdef BLAZERT_PARALLEL_BUILD
#define BLAZERT_MIN_PRIMITIVES_FOR_PARALLEL_BUILD (1024 * 8)
#define BLAZERT_SHALLOW_DEPTH (4)// will create 2**N subtrees
#define BLAZERT_MAX_THREADS (256)
#endif

#if __cplusplus >= 201103L && defined(BLAZERT_PARALLEL_BUILD)
#ifndef BLAZERT_PARALLEL_BUILD_THREADS
#define BLAZERT_PARALLEL_BUILD_THREADS
#endif
#endif

// Prefer OpenMP over c++11 threads
#if defined(_OPENMP) && defined(BLAZERT_PARALLEL_BUILD)
#undef BLAZERT_PARALLEL_BUILD_THREADS
#ifndef BLAZERT_PARALLEL_BUILD_OPENMP
#define BLAZERT_PARALLEL_BUILD_OPENMP
#endif
#endif

#endif// BLAZERT_DEFINES_H_
