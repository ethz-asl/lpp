cmake_minimum_required(VERSION 3.5)
project(lpp VERSION 0.1.0 LANGUAGES CXX)

# Set standard of top level project or C++17
if (NOT DEFINED CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
        if (ENABLE_COVERAGE) # Do not enable -Werror for coverage builds
                add_definitions(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
        else ()
                add_definitions(-Wall -Wextra -Wpedantic -Werror -Wno-unknown-pragmas)
        endif ()
else ()
        set(CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
endif ()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

get_directory_property(hasParent PARENT_DIRECTORY)

# Set LPP_INCLUDE_DIRS in the top level CMake
if(hasParent)
        set(LPP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include PARENT_SCOPE)
else()
        set(LPP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include)
endif()


include(GNUInstallDirs)
include(CMakePackageConfigHelpers)


add_library(lpp INTERFACE)
add_library(lpp::lpp ALIAS lpp)

find_package(glog)

# As there are no src files, set the linker language explicitly
set_target_properties(lpp PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(lpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../include/log++.h"
        DESTINATION include)

# Export + install the INTERFACE target
install(TARGETS lpp
  EXPORT lppTargets
  INCLUDES DESTINATION include
)

install(EXPORT lppTargets
  NAMESPACE lpp::
  FILE  lppTargets.cmake
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/lpp"
)

# Generate and install LPPConfig.cmake + version
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/lppConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/lppConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/lpp"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/lppConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/lppConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/lppConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/lpp"
)
