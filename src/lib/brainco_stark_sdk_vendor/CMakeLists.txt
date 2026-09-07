cmake_minimum_required(VERSION 3.22)
project(brainco_stark_sdk_vendor)

find_package(ament_cmake REQUIRED)

# The BrainCo Stark SDK is distributed only as a prebuilt binary zip, so there is
# nothing to compile here: this package downloads the release for the current
# platform and installs it, which is what makes it visible to colcon's install
# tree instead of living in a source-tree vendor/ directory.
set(BRAINCO_STARK_SDK_VERSION "v1.0.0"
  CACHE STRING "bc-stark-sdk release tag to download (e.g. v1.0.0, v2.0.2)")
set(BRAINCO_STARK_SDK_BASE_URL "https://app.brainco.cn/universal/bc-stark-sdk/libs"
  CACHE STRING "Base URL the SDK release zips are served from")
set(BRAINCO_STARK_SDK_DOWNLOAD_TIMEOUT 120
  CACHE STRING "Seconds to wait for the SDK download before failing")

# --- platform -> release archive name, mirroring scripts/download_sdk.sh -------
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_sdk_platform "linux-arm64")
  else()
    set(_sdk_platform "linux")
  endif()
  set(_sdk_lib_name "libbc_stark_sdk.so")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(_sdk_platform "mac")
  set(_sdk_lib_name "libbc_stark_sdk.dylib")
elseif(WIN32)
  set(_sdk_platform "win")
  set(_sdk_lib_name "bc_stark_sdk.dll")
else()
  message(FATAL_ERROR
    "brainco_stark_sdk_vendor: no SDK release is published for system "
    "'${CMAKE_SYSTEM_NAME}' (processor '${CMAKE_SYSTEM_PROCESSOR}').")
endif()
message(STATUS
  "brainco_stark_sdk_vendor: platform '${_sdk_platform}', "
  "version '${BRAINCO_STARK_SDK_VERSION}'")

set(_sdk_url "${BRAINCO_STARK_SDK_BASE_URL}/${BRAINCO_STARK_SDK_VERSION}/${_sdk_platform}.zip")
set(_sdk_root "${CMAKE_CURRENT_BINARY_DIR}/sdk")
set(_sdk_zip "${CMAKE_CURRENT_BINARY_DIR}/${_sdk_platform}.zip")
# Stamp encodes version and platform, so switching either one forces a re-fetch
# while a plain rebuild reuses what is already on disk.
set(_sdk_stamp "${_sdk_root}/.fetched-${BRAINCO_STARK_SDK_VERSION}-${_sdk_platform}")

# --- fetch --------------------------------------------------------------------
if(EXISTS "${_sdk_stamp}")
  message(STATUS
    "brainco_stark_sdk_vendor: ${BRAINCO_STARK_SDK_VERSION} already unpacked, "
    "skipping download")
else()
  message(STATUS "brainco_stark_sdk_vendor: downloading ${_sdk_url}")
  file(DOWNLOAD "${_sdk_url}" "${_sdk_zip}"
    INACTIVITY_TIMEOUT ${BRAINCO_STARK_SDK_DOWNLOAD_TIMEOUT}
    STATUS _dl_status
    TLS_VERIFY ON
    SHOW_PROGRESS)
  list(GET _dl_status 0 _dl_code)
  list(GET _dl_status 1 _dl_message)
  if(NOT _dl_code EQUAL 0)
    file(REMOVE "${_sdk_zip}")
    message(FATAL_ERROR
      "brainco_stark_sdk_vendor: download failed (${_dl_code}: ${_dl_message})\n"
      "  url: ${_sdk_url}\n"
      "  Check network access, or point BRAINCO_STARK_SDK_VERSION at a release "
      "that exists for platform '${_sdk_platform}'.")
  endif()

  file(REMOVE_RECURSE "${_sdk_root}")
  file(MAKE_DIRECTORY "${_sdk_root}")
  file(ARCHIVE_EXTRACT INPUT "${_sdk_zip}" DESTINATION "${_sdk_root}")
  file(REMOVE "${_sdk_zip}")
  file(REMOVE_RECURSE "${_sdk_root}/__MACOSX")
  file(WRITE "${_sdk_stamp}" "${BRAINCO_STARK_SDK_VERSION} ${_sdk_platform}\n")
  message(STATUS "brainco_stark_sdk_vendor: unpacked into ${_sdk_root}")
endif()

# --- locate what we unpacked --------------------------------------------------
# Globbed rather than hardcoded: the per-platform subdirectory under dist/shared
# is not the same on every release, and a wrong guess would fail at link time
# instead of here.
set(_sdk_include_dir "${_sdk_root}/dist/include")
file(GLOB_RECURSE _sdk_lib_candidates "${_sdk_root}/dist/shared/*/${_sdk_lib_name}")

if(NOT EXISTS "${_sdk_include_dir}/stark-sdk.h")
  message(FATAL_ERROR
    "brainco_stark_sdk_vendor: stark-sdk.h missing after extraction. Expected "
    "'${_sdk_include_dir}/stark-sdk.h' — the archive layout for "
    "${BRAINCO_STARK_SDK_VERSION} may differ from the one this package expects.")
endif()
if(NOT _sdk_lib_candidates)
  message(FATAL_ERROR
    "brainco_stark_sdk_vendor: '${_sdk_lib_name}' not found under "
    "'${_sdk_root}/dist/shared/' after extraction.")
endif()
list(GET _sdk_lib_candidates 0 _sdk_lib_path)
message(STATUS "brainco_stark_sdk_vendor: library at ${_sdk_lib_path}")

# --- install into the colcon tree ---------------------------------------------
# This is the whole point of the package: after install, downstream links against
# ${prefix}/lib and the loader resolves it from LD_LIBRARY_PATH, instead of the
# absolute source-tree RPATH that leaves a dangling reference when the tree moves.
install(
  DIRECTORY "${_sdk_include_dir}/"
  DESTINATION include/${PROJECT_NAME}
)
install(
  FILES "${_sdk_lib_path}"
  DESTINATION lib
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(bc_stark_sdk)

ament_package(
  CONFIG_EXTRAS cmake/brainco_stark_sdk_vendor-extras.cmake.in
)
