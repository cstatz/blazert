Dependencies
------------
blazert itself depends only on the `blaze library <https://bitbucket.org/blaze-lib/blaze/src/master/>`_ for its linear
algebra data types. All other dependencies are only needed for the tests, examples or benchmarks.

===================================================================== =========== ===============================
Dependency                                                             Version      needed for
===================================================================== =========== ===============================
C++ compiler supporting C++17 (e.g. gcc, clang)

CMake                                                                  >= 3.11.0

`blaze library <https://bitbucket.org/blaze-lib/blaze/src/master/>`_   >= 3.7      everything

`embree <https://github.com/embree/embree/>`_                          >= 3        embree fallback, benchmarks

`doctest <https://github.com/onqtam/doctest/>`_                        current     running tests (submodule)

`google benchmark <https://github.com/google/benchmark/>`_             current     running benchmarks
===================================================================== =========== ===============================

.. note:: doctest is included as submodule and does not need to be installed in the system as of now.