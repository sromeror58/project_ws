# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robots_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robots_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robots_FOUND FALSE)
  elseif(NOT robots_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robots_FOUND FALSE)
  endif()
  return()
endif()
set(_robots_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robots_FIND_QUIETLY)
  message(STATUS "Found robots: 0.0.0 (${robots_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robots' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT robots_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robots_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robots_DIR}/${_extra}")
endforeach()
