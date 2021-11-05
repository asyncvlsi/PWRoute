cmake_minimum_required(VERSION 3.16)

project(PWRoute
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

############################################################################
# Detect the installation path of LEF parser and DEF parser,
# and check if the liblef and libdef library can be found or not
############################################################################
message(STATUS "Detecting environment variable LEF_ROOT...")
set(LEF_ROOT "")
if (DEFINED ENV{LEF_ROOT})
    message(STATUS "Environment variable LEF_ROOT detected: " $ENV{LEF_ROOT})
    include_directories($ENV{LEF_ROOT}/include)
    link_directories($ENV{LEF_ROOT}/lib)
    set(LEF_ROOT $ENV{LEF_ROOT})
else ()
    message(STATUS "Environment variable LEF_ROOT not found, using ACT_HOME instead")
    include_directories($ENV{ACT_HOME}/include)
    link_directories($ENV{ACT_HOME}/lib)
    set(LEF_ROOT $ENV{ACT_HOME})
endif ()
find_library(
    LEF_LIBRARY
    NAMES lef
    PATHS ${LEF_ROOT}/lib
    REQUIRED
)
if (NOT LEF_LIBRARY)
    message(FATAL_ERROR "Cannot find liblef.a")
else ()
    message(STATUS "Found liblef.a: " ${LEF_LIBRARY})
endif ()

message(STATUS "Detecting environment variable DEF_ROOT...")
set(DEF_ROOT "")
if (DEFINED ENV{DEF_ROOT})
    message(STATUS "Environment variable DEF_ROOT detected: " $ENV{DEF_ROOT})
    include_directories($ENV{DEF_ROOT}/include)
    link_directories($ENV{DEF_ROOT}/lib)
    set(DEF_ROOT $ENV{DEF_ROOT})
else ()
    message(STATUS "Environment variable DEF_ROOT not found, using ACT_HOME instead")
    include_directories($ENV{ACT_HOME}/include)
    link_directories($ENV{ACT_HOME}/lib)
    set(DEF_ROOT $ENV{ACT_HOME})
endif ()
find_library(
    DEF_LIBRARY
    NAMES def
    PATHS ${DEF_ROOT}/lib
    REQUIRED
)
if (NOT DEF_LIBRARY)
    message(FATAL_ERROR "Cannot find libdef.a")
else ()
    message(STATUS "Found libdef.a: " ${DEF_LIBRARY})
endif ()

message(STATUS "Detecting environment variable $ENV{ACT_HOME}...")
if (DEFINED ENV{ACT_HOME})
    message(STATUS "Environment variable ACT_HOME detected: " $ENV{ACT_HOME})
else ()
    message(FATAL_ERROR "Environment variable $ENV{ACT_HOME} not found")
endif ()
include_directories($ENV{ACT_HOME}/include)
link_directories($ENV{ACT_HOME}/lib)
find_library(PHYDB_LIBRARY
    NAMES phydb
    PATHS $ENV{ACT_HOME}/lib
    REQUIRED)
if (NOT PHYDB_LIBRARY)
    message(FATAL_ERROR "libphydb.a is not found")
else ()
    message(STATUS "libphydb.a found")
endif()

message(STATUS "Detecting Galois libraries...")
set(Galois_FOUND TRUE)
find_library(
    GALOIS_EDA_LIBRARY
    NAMES galois_eda
    PATHS $ENV{ACT_HOME}/lib
)
if (GALOIS_EDA_LIBRARY)
    set(Galois_LIBRARIES ${GALOIS_EDA_LIBRARY})
    message(STATUS "Found libgalois_eda.a: " ${GALOIS_EDA_LIBRARY})
else()
    message(STATUS "Cannot find libgalois_eda.a")
    set(Galois_FOUND FALSE)
endif()
find_library(
    GALOIS_SHMEM_LIBRARY
    NAMES galois_shmem
    PATHS $ENV{ACT_HOME}/lib
)
if (GALOIS_SHMEM_LIBRARY)
    set(Galois_LIBRARIES ${Galois_LIBRARIES} ${GALOIS_SHMEM_LIBRARY})
    message(STATUS "Found libgalois_shmem.a: " ${GALOIS_SHMEM_LIBRARY})
else()
    message(STATUS "Cannot find libgalois_shmem.a")
    set(Galois_FOUND FALSE)
endif()

SET(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.69.0 COMPONENTS log_setup log REQUIRED)
message(STATUS "Boost library path: ${Boost_LIBRARY_DIRS}")
message(STATUS "Boost include path: ${Boost_INCLUDE_DIR}")
message(STATUS "Boost libs: ${Boost_LIBRARIES}")
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Set a default build type if none was specified
set(default_build_type "RELEASE")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}")
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "DEBUG" "RELEASE")
endif ()

add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Set the output directory of static libraries
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Set the output directory of executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

include_directories(include)
FILE(GLOB SOURCES "src/*.cpp")
add_subdirectory(test)

add_library(pwroute 
    STATIC
    ${SOURCES})
target_link_libraries(pwroute
    lef def phydb)

if (BUILD_BIN)
    add_executable(PWRoute test/main.cpp)
    if(Galois_FOUND)
        target_link_libraries(PWRoute 
            phydb pwroute galois_eda galois_shmem pthread numa)
    else ()
        target_link_libraries(PWRoute phydb pwroute)
    endif()
endif ()

######################################
# specify the installation directory: ${ACT_HOME}
######################################

message(STATUS "Detecting environment variable ACT_HOME...")
if (DEFINED ENV{ACT_HOME})
    message(STATUS "Environment variable ACT_HOME detected: " $ENV{ACT_HOME})
else ()
    message(FATAL_ERROR "Environment variable ACT_HOME not found")
endif ()
message(STATUS "Changing Installation directory to ACT_HOME")
set(CMAKE_INSTALL_PREFIX $ENV{ACT_HOME} CACHE PATH "installation path" FORCE)
message(STATUS "Current installation directory: " ${CMAKE_INSTALL_PREFIX})


######################################
# Install header files
######################################
install(DIRECTORY include/ DESTINATION include/pwroute COMPONENT Development FILES_MATCHING PATTERN "*.h")

######################################
# Install library
######################################
install(TARGETS pwroute
    DESTINATION lib)
