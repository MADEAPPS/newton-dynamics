#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenCL::Utils" for configuration "Debug"
set_property(TARGET OpenCL::Utils APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(OpenCL::Utils PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/OpenCLUtilsd.lib"
  )

list(APPEND _cmake_import_check_targets OpenCL::Utils )
list(APPEND _cmake_import_check_files_for_OpenCL::Utils "${_IMPORT_PREFIX}/lib/OpenCLUtilsd.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
