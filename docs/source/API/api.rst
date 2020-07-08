API Reference
=================

Data types
-----------
In order to simplify the usage of blazeRT we have introduced type aliases for blaze vector and matrix types which can
be seen below. In order to have a smooth coding experience, we suggest to use these type aliases.

Vector types
~~~~~~~~~~~~
.. doxygentypedef:: blazert::Vec3r
.. doxygentypedef:: blazert::Vec2r
.. doxygentypedef:: blazert::Vec3ui
.. doxygentypedef:: blazert::Vec2ui


Matrix types
~~~~~~~~~~~~
.. doxygentypedef:: blazert::Mat3r



Container types
~~~~~~~~~~~~~~~~

.. note:: If you define lists of vectors or matrices, you should use the following container aliases.

.. doxygentypedef:: blazert::Vec3rList
.. doxygentypedef:: blazert::Vec3iList
.. doxygentypedef:: blazert::Mat3rList



BlazertScene
------------

.. doxygenclass:: blazert::BlazertScene
    :members:

.. doxygenfunction:: blazert::intersect1


Ray & RayHit
-------------
.. doxygenclass:: blazert::Ray
    :members:

.. doxygenstruct:: blazert::RayHit
    :members:

