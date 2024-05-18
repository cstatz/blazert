# emrtpo
This is a raytracing / (geometric optics) - physical optics hybrid code for the computation of radar signatures.

## Notes regarding blazert
 * The normal computation in blazert trimesh is broken.

## Building
 * Copy into examples subdir of blazert
 * Add em_tracer to CMakeLists.txt in examples subdir of blazert
 * Build blazert

## Usage
 * Generate input files using the simulation_input.ipynb
 * run emrtpo_bistatic_tracer with the created json
 * run bistatic_*_po with the created json.
 * read the results with the read_results.ipynb
