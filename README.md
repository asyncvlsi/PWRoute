# Power router for gridded cell placement

### Build tools
* CMake, version >= 3.16
* GNU Compiler Collection (GCC), version >= 4.8.5

### Pre-requisite libraries
* Boost, version >= 1.61.0
* Si2 LEF/DEF parser: [LEF parser](https://github.com/The-OpenROAD-Project/lef) and [DEF parser](https://github.com/The-OpenROAD-Project/def). Use `cmake .. -DCMAKE_INSTALL_PREFIX=path/to/install` to specify the installation destination for each parser, and export the path as environment variable `LEF_ROOT` and `DEF_ROOT`. This LEF/DEF parser requires BISON (>= 3.0.4)
* [ACT](https://github.com/asyncvlsi/act): environment variable `ACT_HOME` determines the installation destination of this package

### Clone, compile, and install
    $ git clone https://github.com/asyncvlsi/PWRoute.git
    $ cd PWRoute/
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ make install
 "make install" creates a folder `pwroute` under folder `$ACT_HOME/include` with all .h of PWRoute/include, and a static library `libphydb.a` under folder `$ACT_HOME/lib`.

### Notice
* The LEF parser linked above has a memory leak. This LEF parser is a modified version of the original LEF parser from Si2; the original parser does not have this issue, and you can obtain that from Si2 directly.
