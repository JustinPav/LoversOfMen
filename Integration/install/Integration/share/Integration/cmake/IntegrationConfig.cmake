# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Integration_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Integration_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Integration_FOUND FALSE)
  elseif(NOT Integration_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Integration_FOUND FALSE)
  endif()
  return()
endif()
set(_Integration_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Integration_FIND_QUIETLY)
  message(STATUS "Found Integration: 0.0.0 (${Integration_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Integration' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Integration_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Integration_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Integration_DIR}/${_extra}")
endforeach()
