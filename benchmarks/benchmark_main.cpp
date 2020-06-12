//
// Created by ogarten on 06/05/2020.
//

#include <benchmark/benchmark.h>
#include <blaze/Math.h>
#include <blazert/datatypes.h>
#include <iostream>

//BENCHMARK_MAIN();

int main(int argc, char **argv) {

  using embreeVec3 =
      blaze::StaticVector<float, 3UL, blaze::columnVector, blaze::AlignmentFlag::aligned, blaze::PaddingFlag::padded>;
  using embreeVec3ui = blaze::StaticVector<unsigned int, 3UL, blaze::columnVector, blaze::AlignmentFlag::aligned,
                                           blaze::PaddingFlag::padded>;

  std::cout << "sizeof(embreeVec3)    = " << sizeof(embreeVec3) << "\n"
            << "sizeof(embreVec3ui)   = " << sizeof(embreeVec3ui) << "\n"
            << "sizeof(Vec3r<float>)  = " << sizeof(blazert::Vec3r<float>) << "\n"
            << "sizeof(Vec3r<double>) = " << sizeof(blazert::Vec3r<double>) << "\n"
            << "sizeof(Vec3ui)        = " << sizeof(blazert::Vec3ui) << "\n";

  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv))
    return 1;

  benchmark::RunSpecifiedBenchmarks();
}