cmake_minimum_required(VERSION 3.0.2)
project(lpp)

# Don't build tests when using as a submodule
if (${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
        set(LPP_BUILD_TESTS 1)
endif ()

# Set standard of top level project or C++17
if (NOT DEFINED ${CMAKE_CXX_STANDARD})
        set(CMAKE_CXX_STANDARD 17)
        add_definitions(-Wall -Wextra -Wpedantic -Werror -Wno-unknown-pragmas)
else ()
        set(CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
endif ()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

get_directory_property(hasParent PARENT_DIRECTORY)

# Set LPP_INCLUDE_DIRS in the top level CMake
if(hasParent)
        set(LPP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
else()
        set(LPP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

find_package(glog)
find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(catkin_FOUND)
        catkin_package(
                INCLUDE_DIRS include
                LIBRARIES lpp
                CATKIN_DEPENDS roscpp
        )
endif()
add_library(lpp ${CMAKE_CURRENT_SOURCE_DIR}/include/log++.h)
# As there are no src files, set the linker language explicitly
set_target_properties(lpp PROPERTIES LINKER_LANGUAGE CXX)

add_executable(lpp_demo ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp)
target_compile_definitions(lpp_demo PRIVATE MODE_LPP)

if (catkin_FOUND)
        set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${LPP_INCLUDE_DIRS})
        include_directories(${catkin_INCLUDE_DIRS})
        target_link_libraries(lpp_demo ${catkin_LIBRARIES})

        install(TARGETS lpp
                ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
                )
endif ()

if (GLOG_FOUND)
        target_link_libraries(lpp_demo glog)
endif ()

################## Testing ##################

if (NOT LPP_BUILD_TESTS)
        message(STATUS "Skip Log++ tests")
endif()
# All logging frameworks are required for unittests
if (GLOG_FOUND AND catkin_FOUND AND LPP_BUILD_TESTS)

        message(STATUS "Found glog and catkin: Log++ tests will be built")
        ##### Common includes #####
        set(LPP_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/common)

        ##### GLOG Tests #####
        set(GLOG_TESTS "test_glog")

        add_executable(${GLOG_TESTS} test/test_entry_point.cpp
                test/glog/test_glog_basic.cc
                test/glog/test_glog_every_n.cc
                test/glog/test_glog_first_n.cc
                test/glog/test_glog_if_every_n.cc
                test/glog/test_glog_log_string.cc
                test/glog/test_glog_rosprintf.cc
                test/glog/test_glog_vlog.cc)

        target_include_directories(${GLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/glog)
        target_link_libraries(${GLOG_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${GLOG_TESTS} PRIVATE MODE_GLOG)
        target_compile_options(${GLOG_TESTS} PRIVATE "-fcompare-debug-second")

        ##### Log++ Tests #####
        set(LPP_TESTS "test_lpp")
        add_executable(${LPP_TESTS} test/test_entry_point.cpp
                test/lpp/test_lpp_basic.cc
                test/lpp/test_lpp_every_n.cc
                test/lpp/test_lpp_first_n.cc
                test/lpp/test_lpp_if_every_n.cc
                test/lpp/test_lpp_log_string.cc
                test/lpp/test_lpp_rosprintf.cc
                test/lpp/test_lpp_vlog.cc)

        target_include_directories(${LPP_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/lpp)
        target_link_libraries(${LPP_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${LPP_TESTS} PRIVATE MODE_LPP)
        target_compile_options(${LPP_TESTS} PRIVATE "-fcompare-debug-second")

        ##### Log++ Custom Output Tests #####
        set(LPP_TESTS_CUSTOM "test_lpp_custom")
        add_executable(${LPP_TESTS_CUSTOM}
                test/test_entry_point.cpp
                test/lpp/custom/test_lpp_custom_basic.cc
                test/lpp/custom/test_lpp_custom_every_n.cc
                test/lpp/custom/test_lpp_custom_first_n.cc
                test/lpp/custom/test_lpp_custom_if_every_n.cc
                test/lpp/custom/test_lpp_custom_log_string.cc
                test/lpp/custom/test_lpp_custom_rosprintf.cc
                test/lpp/custom/test_lpp_custom_vlog.cc)

        target_include_directories(${LPP_TESTS_CUSTOM} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/lpp)
        target_link_libraries(${LPP_TESTS_CUSTOM} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${LPP_TESTS_CUSTOM} PRIVATE MODE_LPP)
        target_compile_options(${LPP_TESTS_CUSTOM} PRIVATE "-fcompare-debug-second")

        ##### Roslog Tests #####
        set(ROSLOG_TESTS "test_roslog")
        add_executable(${ROSLOG_TESTS} test/test_entry_point.cpp
                test/roslog/test_roslog_basic.cc
                test/roslog/test_roslog_every_n.cc
                test/roslog/test_roslog_first_n.cc
                test/roslog/test_roslog_if_every_n.cc
                test/roslog/test_roslog_log_string.cc
                test/roslog/test_roslog_rosprintf.cc
                test/roslog/test_roslog_vlog.cc)

        target_include_directories(${ROSLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/roslog)
        target_link_libraries(${ROSLOG_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${ROSLOG_TESTS} PRIVATE MODE_ROSLOG)
        target_compile_options(${ROSLOG_TESTS} PRIVATE "-fcompare-debug-second")

        ##### Default Tests #####
        set(DEFAULT_TESTS "test_default")
        add_executable(${DEFAULT_TESTS} test/test_entry_point.cpp
                test/default/test_common.cc
                test/default/test_default_basic.cc
                test/default/test_default_every_n.cc
                test/default/test_default_first_n.cc
                test/default/test_default_if_every_n.cc
                test/default/test_default_log_string.cc
                test/default/test_default_rosprintf.cc
                test/default/test_default_timed.cc
                test/default/test_default_vlog.cc
                test/default/test_severity_conversions.cc)

        target_include_directories(${DEFAULT_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/default)
        target_link_libraries(${DEFAULT_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${DEFAULT_TESTS} PRIVATE MODE_DEFAULT)
        target_compile_options(${DEFAULT_TESTS} PRIVATE "-fcompare-debug-second")

        ##### Nolog Tests #####
        set(NOLOG_TESTS "test_nolog")
        add_executable(${NOLOG_TESTS} test/test_entry_point.cpp
                test/nolog/test_nolog_basic.cc
                test/nolog/test_nolog_every_n.cc
                test/nolog/test_nolog_first_n.cc
                test/nolog/test_nolog_if_every_n.cc
                test/nolog/test_nolog_log_string.cc
                test/nolog/test_nolog_rosprintf.cc
                test/nolog/test_nolog_timed.cc
                test/nolog/test_nolog_vlog.cc
        )

        target_include_directories(${NOLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/nolog)
        target_link_libraries(${NOLOG_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${NOLOG_TESTS} PRIVATE MODE_NOLOG)
        target_compile_options(${NOLOG_TESTS} PRIVATE "-fcompare-debug-second")
endif ()
