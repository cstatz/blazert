Build on Linux
==============

For windows read this: :doc:`build_windows`.

This is a header-only library. No need to build anything. Just drop it in your source directory and off you go.
The build step is solely for the examples, tests and the benchmark.

Prerequisites
-------------



Dependencies
------------

Build
-----
We strictly recommend an out-of-source build in a separate directory (here for simplicity ``build``)
Starting in the source directory to project is build from the commandline as follows:

.. code-block:: shell

    mkdir build
    cd build
    ccmake ../  # create cache and configuration
    cmake --build .
    cmake --build . -- install  # If package needs to be installed
    ctest  # Runs the tests

Notes
-----

.. note:: **For maximum performance**, we recommend building with **gcc** which results in a 15% to 20% better performance compared to clang (on linux and macOS). The provided benchmarks might be used to tune the compilation flags for your specific system and architecture.



.. note:: **A word of caution:** blazeRT will compile and work with compiler optimizations enabled (up to **-O3**), but needs infinite-math. If your application needs fast-math, ensure that the blazeRT code path is compiled with `-fno-finite-math-only` (in case of clang). In terms of performance, in its current form there is no major runtime difference between compilation with *-O2* and *-O3*.
