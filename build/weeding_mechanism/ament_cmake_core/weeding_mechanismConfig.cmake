# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_weeding_mechanism_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED weeding_mechanism_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(weeding_mechanism_FOUND FALSE)
  elseif(NOT weeding_mechanism_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(weeding_mechanism_FOUND FALSE)
  endif()
  return()
endif()
set(_weeding_mechanism_CONFIG_INCLUDED TRUE)

# output package information
if(NOT weeding_mechanism_FIND_QUIETLY)
  message(STATUS "Found weeding_mechanism: 0.0.0 (${weeding_mechanism_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'weeding_mechanism' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${weeding_mechanism_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(weeding_mechanism_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${weeding_mechanism_DIR}/${_extra}")
endforeach()
