Building on Windows with `VCPKG <https://github.com/microsoft/vcpkg/>`_
=======================================================================

Prerequisites
-------------

 * git
 * Visual Studio Community 2019

**NOTE:** building instructions should might be executed from ``VS x64 native command prompt`` or ``Powershell``

**NOTE:** install vcpkg to a folder where there are no whitespace in the folder path.

Getting and bootstrapping vcpkg
-------------------------------

Get vcpkg and bootstrap it:

.. code-block:: shell

    git clone https://github.com/microsoft/vcpkg.git
    cd vcpkg.git
    bootstrap-vcpkg.bat
    vcpkg integrate install (run as Administrator on first run)


The last step is not necessary if you don't use the VS IDE.

Install the dependencies
------------------------

Install the necessary dependencies for blazert:

.. code-block:: shell

    vcpkg install blaze:x64-windows
    vcpkg install embree:x64-windows
    vcpkg install benchmark:x64-windows



Get and build blazeRT
---------------------

Create a project (and/or build):

.. code-block:: shell

    git clone https://github.com/cstatz/blazert.git
    cd blazert
    mkdir build && cd build
    cmake -G "Visual Studio 16 2019 Win64" \
        -DCMAKE_TOOLCHAIN_FILE=[vcpkg root]\scripts\buildsystems\vcpkg.cmake \
        -DBUILD_EXAMPLES:BOOL=true \
        -DBLAZE_INCLUDE_OVERRIDE:STRING=[vcpkg root]\installed\x64-windows\include \
        ..

    cmake --build .
