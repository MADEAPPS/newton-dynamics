#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenCL::OpenCL" for configuration "Debug"
set_property(TARGET OpenCL::OpenCL APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(OpenCL::OpenCL PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/OpenCL.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/OpenCL.dll"
  )

list(APPEND _cmake_import_check_targets OpenCL::OpenCL )
list(APPEND _cmake_import_check_files_for_OpenCL::OpenCL "${_IMPORT_PREFIX}/lib/OpenCL.lib" "${_IMPORT_PREFIX}/bin/OpenCL.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
