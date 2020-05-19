//
// Created by ogarten on 06/05/2020.
//

#include <benchmark/benchmark.h>

#include <blazert/primitives/trimesh.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <iostream>
#include <vector>

void run_BVHBuild() {
  int i = 0;
  for (size_t i=0; i < 1000; ++i) {
    std::vector<int> vec{1,2,3};
  }
}

static void
BM_TRAVERSAL(benchmark::State& state)
{
  for (auto _ : state) {
    run_BVHBuild();
  }
}
//BENCHMARK(BM_TRAVERSAL);//->Unit(benchmark::kMillisecond);