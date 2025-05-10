project(healpix)

set(SRC
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
set(libsharp_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libsharp)
set(libsharp_ARTIFACT_DIR ${libsharp_SOURCE_DIR}/auto) # Default output location for libsharp

ExternalProject_Add(libsharp_build_step
  SOURCE_DIR ${libsharp_SOURCE_DIR} # Location of libsharp sources
  CONFIGURE_COMMAND sh -c "autoconf && autoupdate && ./configure" # Configure libsharp
  BUILD_COMMAND ${CMAKE_MAKE_PROGRAM} default # Build libsharp (default target includes hdrcopy)
  INSTALL_COMMAND "" # No separate install step, artifacts are in libsharp_ARTIFACT_DIR
  BUILD_IN_SOURCE 1  # Indicates that the build happens in the source directory
  BUILD_BYPRODUCTS # Helps CMake track when libsharp is built
    "${libsharp_ARTIFACT_DIR}/lib/libsharp.a"
    "${libsharp_ARTIFACT_DIR}/lib/libfftpack.a"
    "${libsharp_ARTIFACT_DIR}/lib/libc_utils.a"
    "${libsharp_ARTIFACT_DIR}/include/sharp.h" # Assuming sharp.h is a key header
)

# Create an interface library to represent the built libsharp
add_library(sharp_libs INTERFACE)
target_include_directories(sharp_libs INTERFACE "${libsharp_ARTIFACT_DIR}/include")
target_link_libraries(sharp_libs INTERFACE
  "${libsharp_ARTIFACT_DIR}/lib/libsharp.a"
  "${libsharp_ARTIFACT_DIR}/lib/libfftpack.a"
  "${libsharp_ARTIFACT_DIR}/lib/libc_utils.a"
)

# Ensure libsharp is built before anything tries to use sharp_libs
add_dependencies(sharp_libs libsharp_build_step)

set(SHARP_LIBRARY_NAME sharp_libs)
# ---- End of libsharp integration ----

add_subdirectory(cfitsio)
set_property(TARGET cfitsio PROPERTY UNITY_BUILD OFF)

add_library(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC healpix/src/cxx/cxxsupport healpix/src/cxx/Healpix_cxx)
# Link healpix against cfitsio and the imported libsharp
target_link_libraries(${PROJECT_NAME} PUBLIC cfitsio ${SHARP_LIBRARY_NAME})