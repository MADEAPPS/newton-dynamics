get_filename_component(PARENT_DIR ${CMAKE_CURRENT_LIST_DIR} PATH)
include("${PARENT_DIR}/OpenCLHeaders/OpenCLHeadersConfig.cmake")
include("${PARENT_DIR}/OpenCLICDLoader/OpenCLICDLoaderConfig.cmake")
include("${PARENT_DIR}/OpenCLHeadersCpp/OpenCLHeadersCppConfig.cmake")
include("${PARENT_DIR}/OpenCLUtils/OpenCLUtilsConfig.cmake")
include("${PARENT_DIR}/OpenCLUtilsCpp/OpenCLUtilsCppConfig.cmake")
  