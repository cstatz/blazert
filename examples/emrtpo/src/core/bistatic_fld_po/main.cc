#include "cnpy.h"
#include <complex>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <chrono>

#include "emrtpo_tracer_config.h"
#include "emrtpo_po_config.h"
#include "numpy_wrapper.h"

#include "jupyter_notebook.h"
#include "transforms.h"

#include "fld_po_physics.h"

using namespace std::complex_literals;

using namespace blazert;
using namespace emrt;


int main(int argc, char **argv) {

  if (argc < 2) {
    std::cout << "Processing config file missing (json)" << std::endl;
    std::cout << "Usage: \n"
              << "binary_read_process_ray <results processing config>\n";
    return 1;
  }

  auto config = VectorEMRTPOResultsConfig<double>(argv[1]);
  std::cout << config << std::endl;
  const size_t n_freq = config.n_frequencies;
  const std::string output_name = config.output_filename;

  if (!config.assert_unambiguous_range)
    std::cerr << "Check for unambiguous range violation is disabled!\n";
  auto start = std::chrono::steady_clock::now();

  // get config for source
  //const std::string emrt_config_path = config; //identifier + ".json";
  //const VectorEMRTPOConfig<double> emrt_config{emrt_config_path.c_str()};

  //std::cout << "EMRT config: \n " << emrt_config << "\n";

  //const std::string temporary_output_fname = output_name + ".npy";

  //const blaze::DynamicVector<Vec3r<std::complex<double>>> field =
  //    process_single_target_position(emrt_config, config, config.pattern_vector, temporary_output_fname);

  auto stop = std::chrono::steady_clock::now();
  auto diff = stop - start;
  std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::seconds>(diff).count() << "s \n";

  //cnpy::npy_save(temporary_output_fname, field.data(), {config.n_frequencies, 3}, "w");
  std::cout << "Done." << std::endl;
  return 0;
}
