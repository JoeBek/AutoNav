#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "zed_interfaces::zed_interfaces__rosidl_generator_py" for configuration ""
set_property(TARGET zed_interfaces::zed_interfaces__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(zed_interfaces::zed_interfaces__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libzed_interfaces__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "libzed_interfaces__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS zed_interfaces::zed_interfaces__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_zed_interfaces::zed_interfaces__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libzed_interfaces__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
