//
// Created by ogarten on 19/05/2020.
//

#include <benchmark/benchmark.h>

#include "../benchmark_helper/OriginSphere.h"

static void
BM_TRAVERSAL(benchmark::State& state)
{
  auto os = OriginSphere(5);
  for (auto _ : state) {
  }
}
BENCHMARK(BM_TRAVERSAL);//->Unit(benchmark::kMillisecond);