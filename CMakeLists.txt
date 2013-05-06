cmake_minimum_required(VERSION 2.8)

project(Newton)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_SOURCE_DIR}
)

file(GLOB_RECURSE NewtonLibSrc
    ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/*.cpp
)

file(GLOB_RECURSE NewtonLibH
    ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/*.h
)


# remove some stuff:
# remove class which does not build in this version
set(to_remove
    ${to_remove}
    ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/physics/dgVehicleConstraint.cpp
)

# remove dllmain in non-windows version
if(NOT WIN32)
set(to_remove
    ${to_remove}
    ${CMAKE_SOURCE_DIR}/packages/dScene/dllmain.cpp
)
endif(NOT WIN32)


# set packages to compile
set(packages
    dAILibrary
    dAnimation
#    dCompilerKit
#    dContainers
    dCustomJoints
    dMath
    dScene
#    dVisualDebuggerServer
)

foreach(package ${packages})
file(GLOB_RECURSE NewtonPackagesSrcTmp
    ${CMAKE_SOURCE_DIR}/packages/${package}/*.cpp
)
set(NewtonPackagesSrc
    ${NewtonPackagesSrc}
    ${NewtonPackagesSrcTmp}
)


file(GLOB_RECURSE NewtonPackagesHTmp
    ${CMAKE_SOURCE_DIR}/packages/${package}/*.h
)
set(NewtonPackagesH
    ${NewtonPackagesH}
    ${NewtonPackagesHTmp}
)
endforeach(package ${packages})

list(REMOVE_ITEM NewtonLibSrc ${to_remove})
list(REMOVE_ITEM NewtonPackagesSrc ${to_remove})


FIND_PACKAGE(TinyXML REQUIRED)


file(GLOB includeDirs
    ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/*/
    ${CMAKE_SOURCE_DIR}/packages/*/
)


include_directories(${includeDirs})
include_directories(${TINYXML_INCLUDE_DIR})

if(UNIX)
add_definitions( -DJOINTLIBRARY_LIB -D_LINUX_VER -D_POSIX_VER)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  add_definitions( -D_LINUX_VER_64 -D_POSIX_VER_64)
endif()

endif(UNIX)


SET(LIB_TYPE SHARED)
add_library(Newton ${LIB_TYPE} ${NewtonLibH} ${NewtonLibSrc}  ${NewtonPackagesH} ${NewtonPackagesSrc})
target_link_libraries(Newton ${TINYXML_LIBRARIES})

INSTALL(TARGETS Newton
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${NewtonLibH} DESTINATION include/newton)

set(CMAKE_CXX_FLAGS
    ${CMAKE_CXX_FLAGS}
    -fno-operator-names
)

if(CMAKE_COMPILER_IS_GNUCC)
set(CMAKE_C_FLAGS
    ${CMAKE_C_FLAGS}
    -fno-operator-names
)
endif()
