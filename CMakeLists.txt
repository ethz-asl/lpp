cmake_minimum_required(VERSION 3.0.2)
project(Log++)

# Don't build tests when using as a submodule
if (${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
        set(LPP_BUILD_TESTS 1)
endif ()

# Set standard of top level project or C++17
if (NOT DEFINED ${CMAKE_CXX_STANDARD})
        set(CMAKE_CXX_STANDARD 17)
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

add_library(Log++ ${CMAKE_CURRENT_SOURCE_DIR}/include/log++.h)
# As there are no src files, set the linker language explicitly
set_target_properties(Log++ PROPERTIES LINKER_LANGUAGE CXX)

add_executable(lpp_demo ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp)
target_compile_definitions(lpp_demo PRIVATE MODE_LPP)

if (catkin_FOUND)
        include_directories(${catkin_INCLUDE_DIRS})
        target_link_libraries(lpp_demo ${catkin_LIBRARIES})
endif ()

if (GLOG_FOUND)
        target_link_libraries(lpp_demo glog)
endif ()

################## Testing ##################

if (NOT LPP_BUILD_TESTS)
        message("-- Skip Log++ tests")
endif()
# All logging frameworks are required for unittests
if (GLOG_FOUND AND catkin_FOUND AND LPP_BUILD_TESTS)

        message("-- Found glog and catkin: Log++ tests will be built")
        ##### Common includes #####
        set(LPP_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/common)

        ##### GLOG Tests #####
        set(GLOG_TESTS "test_glog")

        add_executable(${GLOG_TESTS} test/test_entry_point.cpp
                test/glog/test_glog_basic.cc
                test/glog/test_glog_every_n.cc
                test/glog/test_glog_first_n.cc
                test/glog/test_glog_log_string.cc
                test/glog/test_glog_rosprintf.cc
                test/glog/test_glog_vlog.cc)

        target_include_directories(${GLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/glog)
        target_link_libraries(${GLOG_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${GLOG_TESTS} PRIVATE MODE_GLOG)

        ##### Log++ Tests #####
        set(LPP_TESTS "test_lpp")
        add_executable(${LPP_TESTS} test/test_entry_point.cpp
                test/lpp/test_lpp_basic.cc
                test/lpp/test_lpp_every_n.cc
                test/lpp/test_lpp_first_n.cc
                test/lpp/test_lpp_log_string.cc
                test/lpp/test_lpp_rosprintf.cc
                test/lpp/test_lpp_vlog.cc)

        target_include_directories(${LPP_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/lpp)
        target_link_libraries(${LPP_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${LPP_TESTS} PRIVATE MODE_LPP)

        ##### Log++ Custom Output Tests #####
        set(LPP_TESTS_CUSTOM "test_lpp_custom")
        add_executable(${LPP_TESTS_CUSTOM}
                test/test_entry_point.cpp
                test/lpp/custom/test_lpp_custom_basic.cc
                test/lpp/custom/test_lpp_custom_every_n.cc
                test/lpp/custom/test_lpp_custom_first_n.cc
                test/lpp/custom/test_lpp_custom_log_string.cc
                test/lpp/custom/test_lpp_custom_rosprintf.cc
                test/lpp/custom/test_lpp_custom_vlog.cc)

        target_include_directories(${LPP_TESTS_CUSTOM} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/lpp)
        target_link_libraries(${LPP_TESTS_CUSTOM} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${LPP_TESTS_CUSTOM} PRIVATE MODE_LPP)

        ##### Roslog Tests #####
        set(ROSLOG_TESTS "test_roslog")
        add_executable(${ROSLOG_TESTS} test/test_entry_point.cpp
                test/roslog/test_roslog_basic.cc
                test/roslog/test_roslog_every_n.cc
                test/roslog/test_roslog_first_n.cc
                test/roslog/test_roslog_log_string.cc
                test/roslog/test_roslog_rosprintf.cc
                test/roslog/test_roslog_vlog.cc)

        target_include_directories(${ROSLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/roslog)
        target_link_libraries(${ROSLOG_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${ROSLOG_TESTS} PRIVATE MODE_ROSLOG)

        ##### Default Tests #####
        set(DEFAULT_TESTS "test_default")
        add_executable(${DEFAULT_TESTS} test/test_entry_point.cpp
                test/default/test_common.cc
                test/default/test_default_basic.cc
                test/default/test_default_every_n.cc
                test/default/test_default_first_n.cc
                test/default/test_default_log_string.cc
                test/default/test_default_rosprintf.cc
                test/default/test_default_timed.cc
                test/default/test_default_vlog.cc
                test/default/test_severity_conversions.cc)

        target_include_directories(${DEFAULT_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/default)
        target_link_libraries(${DEFAULT_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${DEFAULT_TESTS} PRIVATE MODE_DEFAULT)
endif ()
