# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_planner_visualization_demo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED planner_visualization_demo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(planner_visualization_demo_FOUND FALSE)
  elseif(NOT planner_visualization_demo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(planner_visualization_demo_FOUND FALSE)
  endif()
  return()
endif()
set(_planner_visualization_demo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT planner_visualization_demo_FIND_QUIETLY)
  message(STATUS "Found planner_visualization_demo: 0.0.0 (${planner_visualization_demo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'planner_visualization_demo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${planner_visualization_demo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(planner_visualization_demo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${planner_visualization_demo_DIR}/${_extra}")
endforeach()
