# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_assembly_with_links_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED assembly_with_links_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(assembly_with_links_FOUND FALSE)
  elseif(NOT assembly_with_links_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(assembly_with_links_FOUND FALSE)
  endif()
  return()
endif()
set(_assembly_with_links_CONFIG_INCLUDED TRUE)

# output package information
if(NOT assembly_with_links_FIND_QUIETLY)
  message(STATUS "Found assembly_with_links: 0.0.1 (${assembly_with_links_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'assembly_with_links' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${assembly_with_links_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(assembly_with_links_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${assembly_with_links_DIR}/${_extra}")
endforeach()
