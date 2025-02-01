# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_stonefish_bluerov2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED stonefish_bluerov2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(stonefish_bluerov2_FOUND FALSE)
  elseif(NOT stonefish_bluerov2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(stonefish_bluerov2_FOUND FALSE)
  endif()
  return()
endif()
set(_stonefish_bluerov2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT stonefish_bluerov2_FIND_QUIETLY)
  message(STATUS "Found stonefish_bluerov2: 0.0.0 (${stonefish_bluerov2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'stonefish_bluerov2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${stonefish_bluerov2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(stonefish_bluerov2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${stonefish_bluerov2_DIR}/${_extra}")
endforeach()
