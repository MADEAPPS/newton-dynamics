#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenCL::UtilsCpp" for configuration "Debug"
set_property(TARGET OpenCL::UtilsCpp APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(OpenCL::UtilsCpp PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C;CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/OpenCLUtilsCppd.lib"
  )

list(APPEND _cmake_import_check_targets OpenCL::UtilsCpp )
list(APPEND _cmake_import_check_files_for_OpenCL::UtilsCpp "${_IMPORT_PREFIX}/lib/OpenCLUtilsCppd.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
