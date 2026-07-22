# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fleet_cmd_vel_publisher_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fleet_cmd_vel_publisher_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fleet_cmd_vel_publisher_FOUND FALSE)
  elseif(NOT fleet_cmd_vel_publisher_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fleet_cmd_vel_publisher_FOUND FALSE)
  endif()
  return()
endif()
set(_fleet_cmd_vel_publisher_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fleet_cmd_vel_publisher_FIND_QUIETLY)
  message(STATUS "Found fleet_cmd_vel_publisher: 0.1.0 (${fleet_cmd_vel_publisher_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fleet_cmd_vel_publisher' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fleet_cmd_vel_publisher_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fleet_cmd_vel_publisher_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fleet_cmd_vel_publisher_DIR}/${_extra}")
endforeach()
