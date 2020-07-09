Configuration
=============

The library can be configured with various CMAKE options in order to generate the
desired result.

The configuration options can be passed to cmake via command-line with
``cmake -D<OPTION>:<OPTION_TYPE>=<VALUE> ..``
or via the cmake curses interface
``ccmake ..``.

The available options and a short description is listed below.

========================== ===============================
CMAKE Option                DECSRIPTION
========================== ===============================
``BUILD_TEST``              Build test cases.

``BUILD_BENCHMARK``         Build benchmarks.

``BUILD_EXAMPLES``          Build examples.

``ENABLE_OMP``              Enable OpenMP in examples.

``EMBREE_BACKEND``          Embree fallback for scene API.
========================== ===============================

For examples, if you want to build the examples with OpenMP support, run

.. code-block:: shell

    cmake -DBUILD_EXAMPLES:BOOL=true -DENABLE_OMP:BOOL=true <path/to/srcdir>


