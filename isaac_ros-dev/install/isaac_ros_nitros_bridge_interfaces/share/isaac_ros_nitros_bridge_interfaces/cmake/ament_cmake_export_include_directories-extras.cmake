# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "${isaac_ros_nitros_bridge_interfaces_DIR}/../../../include/isaac_ros_nitros_bridge_interfaces")

# append include directories to isaac_ros_nitros_bridge_interfaces_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'isaac_ros_nitros_bridge_interfaces' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND isaac_ros_nitros_bridge_interfaces_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()
