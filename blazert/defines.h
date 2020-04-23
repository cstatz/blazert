#pragma once
#ifndef BLAZERT_DEFINES_H_
#define BLAZERT_DEFINES_H_

// Some constants
#define kBLAZERT_MAX_STACK_DEPTH (512)
#define kBLAZERT_MIN_PRIMITIVES_FOR_PARALLEL_BUILD (1024 * 8)
#define kBLAZERT_SHALLOW_DEPTH (4)// will create 2**N subtrees
#define kBLAZERT_MAX_THREADS (256)

#if __cplusplus >= 201103L
#ifndef BLAZERT_ENABLE_PARALLEL_BUILD
#define BLAZERT_ENABLE_PARALLEL_BUILD
#endif
#endif

#endif// BLAZERT_DEFINES_H_
