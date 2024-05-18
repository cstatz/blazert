// read a file into memory
#include "examples/em_tracer/src/third_party/cnpy.h"
#include <complex>
#include <iostream>
#include <string>
#include <chrono>

#include "emrtpo_po_config.h"
#include "emrtpo_tracer_config.h"
#include "numpy_wrapper.h"

#include "transforms.h"

#include "pattern_config.h"
#include "pattern_po_physics.h"

using namespace std::complex_literals;
using namespace blazert;


int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Processing config file missing (json)" << std::endl;
    std::cout << "Usage: \n"
              << "binary_read_process_ray <results processing config>" << std::endl;
    return 1;
  }

  auto config = VectorEMRTPOResultsConfig<double>(argv[1]);
  std::cout << config << std::endl;

  const std::string output_name = config.output_filename;

  auto pattern_config = PatternConfig<double>(argv[1]);

  std::cout << "Pattern Config: \n" << pattern_config << std::endl;
  // load pattern
  auto origin_pattern = read_2d_array<Vec3rList<std::complex<double>>, std::complex<double>>(pattern_config.transmitter_pattern_fname);
  auto target_pattern = read_2d_array<Vec3rList<std::complex<double>>, std::complex<double>>(pattern_config.receiver_pattern_fname);

  if (config.assert_unambiguous_range)
    std::cerr << "Check for unambiguous range violation is disabled!" << std::endl;
  auto start = std::chrono::steady_clock::now();

  // get config for source
  const std::string emrt_config_path = config.emrtpo_config;
  const VectorEMRTPOConfig<double> emrt_config{emrt_config_path.c_str()};

  std::cout << "EMRT config: \n " << emrt_config << std::endl;

  const std::string output_fname = output_name + ".npy";

  const blaze::DynamicVector<std::complex<double>> field =
      process_single_target_position(emrt_config, config, *origin_pattern, output_fname);

  auto stop = std::chrono::steady_clock::now();
  auto diff = stop - start;

  std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::seconds>(diff).count() << "s \n";
  std::cout << "Done." << std::endl;
  return 0;
}
