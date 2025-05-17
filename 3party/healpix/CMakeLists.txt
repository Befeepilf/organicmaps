project(healpix)

set(SRC
  # healpix/src/common_libraries/libsharp/c_utils/c_utils.c
  # healpix/src/common_libraries/libsharp/c_utils/memusage.c
  # healpix/src/common_libraries/libsharp/c_utils/walltime_c.c

  # healpix/src/common_libraries/libsharp/pocketfft/pocketfft.c
  
  healpix/src/cxx/cxxsupport/announce.cc
  healpix/src/cxx/cxxsupport/geom_utils.cc
  healpix/src/cxx/cxxsupport/string_utils.cc
  healpix/src/cxx/cxxsupport/ls_image.cc
  healpix/src/cxx/cxxsupport/paramfile.cc
  healpix/src/cxx/cxxsupport/pointing.cc
  healpix/src/cxx/cxxsupport/rotmatrix.cc
  healpix/src/cxx/cxxsupport/trafos.cc
  healpix/src/cxx/cxxsupport/walltimer.cc
  healpix/src/cxx/cxxsupport/wigner.cc
  healpix/src/cxx/cxxsupport/error_handling.cc
  healpix/src/cxx/cxxsupport/fitshandle.cc
  healpix/src/cxx/Healpix_cxx/alm.cc
  healpix/src/cxx/Healpix_cxx/alm_healpix_tools.cc
  healpix/src/cxx/Healpix_cxx/alm_powspec_tools.cc
  healpix/src/cxx/Healpix_cxx/healpix_tables.cc
  healpix/src/cxx/Healpix_cxx/healpix_base.cc
  healpix/src/cxx/Healpix_cxx/healpix_map.cc
  healpix/src/cxx/Healpix_cxx/powspec.cc
  healpix/src/cxx/Healpix_cxx/moc_query.cc
  healpix/src/cxx/Healpix_cxx/weight_utils.cc
  healpix/src/cxx/Healpix_cxx/mask_tools.cc
  healpix/src/cxx/Healpix_cxx/alice3.cc
  healpix/src/cxx/Healpix_cxx/healpix_data_io.cc
  healpix/src/cxx/Healpix_cxx/healpix_map_fitsio.cc
  healpix/src/cxx/Healpix_cxx/alm_fitsio.cc
  healpix/src/cxx/Healpix_cxx/powspec_fitsio.cc
  healpix/src/cxx/Healpix_cxx/moc_fitsio.cc
)

set(TESTS OFF)

include(ExternalProject)

# ---- Integration of libsharp build ----
# Build libsharp separately for every Android ABI.  Using an ABI-specific
# out-of-source build directory prevents the artefacts produced for the first
# ABI from being re-used when CMake subsequently configures another ABI
# (which would lead to  "is incompatible with armelf_linux_eabi" or similar
# linker errors).

set(libsharp_CONFIGURE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/healpix/src/common_libraries/libsharp)

# `ANDROID_ABI` is defined by CMake when cross-compiling with the NDK.
# If we are not on Android just fall back to a generic build dir.
if(ANDROID)
  set(_libsharp_build_suffix "${ANDROID_ABI}")
else()
  set(_libsharp_build_suffix "host")
endif()

# The actual build will happen in a separate directory under the main build
# tree to keep the source tree clean.
set(libsharp_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/libsharp_build_${_libsharp_build_suffix}")
set(libsharp_ARTIFACT_DIR "${libsharp_BUILD_DIR}/.libs")

# Compose the compiler flags that CMake already figured out for us and add the
# bits that Autotools normally discovers by itself (position independent code
# and the desired C standard).  We also have to add the explicit target for the
# standalone Clang driver coming from the NDK because Autotools will call the
# compiler without the extra flags that CMake usually injects for us.

set(_LIBSHARP_CFLAGS "${CMAKE_C_FLAGS} -fPIC -std=c99 -D__ANDROID_API__=23 -target ${CMAKE_C_COMPILER_TARGET} -fopenmp")
set(_LIBSHARP_LDFLAGS "-target ${CMAKE_C_COMPILER_TARGET} -fopenmp")

# Autotools detects cross-compilation from the --host option.  Do NOT surround
# the triple with quotes because that will be treated as part of the value and
# break the test programs.

ExternalProject_Add(libsharp_build_step
  SOURCE_DIR ${libsharp_CONFIGURE_DIR}
  BINARY_DIR ${libsharp_BUILD_DIR}

  CONFIGURE_COMMAND
    ${CMAKE_COMMAND} -E chdir ${libsharp_CONFIGURE_DIR}
      sh -c "rm -f config.cache && autoreconf -i"
  COMMAND
    ${CMAKE_COMMAND} -E env
      CC=${CMAKE_C_COMPILER}
      AR=${CMAKE_AR}
      RANLIB=${CMAKE_RANLIB}
      CFLAGS=${_LIBSHARP_CFLAGS}
      LDFLAGS=${_LIBSHARP_LDFLAGS}
      ${libsharp_CONFIGURE_DIR}/configure
        --host=${CMAKE_C_COMPILER_TARGET}
        --enable-pic
        --enable-openmp
        --disable-shared
        --enable-static

  BUILD_COMMAND
    ${CMAKE_COMMAND} -E env
      CC=${CMAKE_C_COMPILER}
      AR=${CMAKE_AR}
      RANLIB=${CMAKE_RANLIB}
      CFLAGS=${_LIBSHARP_CFLAGS}
      LDFLAGS=${_LIBSHARP_LDFLAGS}
      make -j V=1

  INSTALL_COMMAND ""
  BUILD_IN_SOURCE FALSE
  BUILD_BYPRODUCTS
    "${libsharp_ARTIFACT_DIR}/libsharp.a"
    # "${libsharp_ARTIFACT_DIR}/libfftpack.a"
    # "${libsharp_ARTIFACT_DIR}/libc_utils.a"
    # "${libsharp_ARTIFACT_DIR}/include/sharp.h"
)

add_library(sharp_libs INTERFACE)
target_include_directories(sharp_libs INTERFACE "${libsharp_ARTIFACT_DIR}/include")
target_link_libraries(sharp_libs INTERFACE
  "${libsharp_ARTIFACT_DIR}/libsharp.a"
  # "${libsharp_ARTIFACT_DIR}/lib/libfftpack.a"
  # "${libsharp_ARTIFACT_DIR}/lib/libc_utils.a"
)

add_dependencies(sharp_libs libsharp_build_step)

set(SHARP_LIBRARY_NAME sharp_libs)
# ---- End of libsharp integration ----

set(BUILD_SHARED_LIBS OFF)
add_subdirectory(cfitsio)
set_property(TARGET cfitsio PROPERTY UNITY_BUILD OFF)

add_library(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC healpix/src/common_libraries/libsharp healpix/src/common_libraries/libsharp/libsharp healpix/src/cxx/cxxsupport healpix/src/cxx/Healpix_cxx)

# target_include_directories(${PROJECT_NAME} PUBLIC healpix/src/common_libraries/libsharp healpix/src/common_libraries/libsharp/libsharp healpix/src/C/subs)
target_link_libraries(${PROJECT_NAME} PUBLIC cfitsio ${SHARP_LIBRARY_NAME})