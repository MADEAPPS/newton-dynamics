#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenCL::OpenCLExt" for configuration "Release"
set_property(TARGET OpenCL::OpenCLExt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenCL::OpenCLExt PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/OpenCLExt.lib"
  )

list(APPEND _cmake_import_check_targets OpenCL::OpenCLExt )
list(APPEND _cmake_import_check_files_for_OpenCL::OpenCLExt "${_IMPORT_PREFIX}/lib/OpenCLExt.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
