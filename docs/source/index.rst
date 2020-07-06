.. blazert documentation master file, created by
   sphinx-quickstart on Fri Jul  3 16:02:04 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to blazert's documentation!
===================================

This is the official documentation for `blazert <https::/github/cstatz/blazert/>`_ and will be continuously updated.

.. toctree::
   :maxdepth: 2

   build_blazert/index
   usage/index
   API/api

blazeRT is a **double precision ray tracer** for scientific or engineering applications derived from
`nanoRT <https://github.com/lighttransport/nanort/>`_ using blaze datatypes and written in modern C++17.
blazeRTs scene interface is similar to `embree <https://github.com/embree/embree/>`_ and intents to be a minimal
effort (nearly plugin-) replacement. blazeRT should work on any system and architecture for which a recent
(C++17 compatible) compiler is available.

We aim at providing a **simple and unambiguous high-level API** for the ray-traversal.
We do not aim at providing backwards-compatibility (especially to older C++ standards).

blazeRT makes use of the the `blaze <https://bitbucket.org/blaze-lib/blaze/src/master/>`_ linear algebra
library for its vector types. Because we rely on a well-tested and well-optimized linear algebra library
(instead of using our own vector types), blazeRT can focus on the actual ray tracing algorithms. Furthermore,
using types from a linear algebra library is advantageous for the subsequent development of scientific application
where the vector types are needed again. blazeRT should work with any library providing these vector types as
long as certain criteria are met (a minimal set of operation on these vector types).

blazeRT works with triangular meshes and simple primitives, but it is easy to extend blazeRT
to work on polygons or more complex primitives. A template for user-defined geometries can be found
`here <https://github.com/cstatz/blazert/blob/master/examples/geometry_template/GEOM_TEMPLATE.h/>`_. If you implement new geometries, we are more than happy to receive
a pull request from you to include it in blazeRT.

blazeRT is tested using unit tests (whose number will increase as development progresses) as well as by comparison of rendering results to reference images. Currently the unit tests cover roughly 90% of files and 69% of lines, but in the tests we try to catch as many (fringe-) cases as possible.  We try to ensure high code
quality and a reproducible build experience via continuous integration. During the CI process we
build the examples and the tests, which need to run successfully in order for the CI to pass. Currently,
blazeRT is CI-tested on Ubuntu 18.04 and macOS with gcc and clang.

The documentation of the entire script can be found `here <https://blazert.readthedocs.io/en/latest/>`_.

.. image:: ../../examples/baseline/path_tracer_blaze.png

(Rendered using the `path_tracer` example adapted from `nanoRT` and originally contributed by
`daseyb <https://github.com/daseyb/>`_)


Indices and tables
-------------------
* :ref:`genindex`
* :ref:`search`

