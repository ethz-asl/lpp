cmake_minimum_required(VERSION 3.16)
project(Log++)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(glog)
find_package(catkin REQUIRED COMPONENTS roscpp)

add_library(Log++ include/log++.h)
# As there are no src files, set the linker language explicitly
set_target_properties(Log++ PROPERTIES LINKER_LANGUAGE CXX)

add_executable(lpp_demo src/demo.cpp)
target_compile_definitions(lpp_demo PRIVATE MODE_LPP)

if (catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})
    target_link_libraries(lpp_demo ${catkin_LIBRARIES})
endif ()

if (GLOG_FOUND)
    target_link_libraries(lpp_demo glog)
endif ()

################## Testing ##################

# All logging frameworks are required for unittests
if (GLOG_FOUND)
    if (catkin_FOUND)

        message("-- Found glog and catkin: Log++ tests will be built")
        ##### Common includes #####
        set(LPP_INCLUDE_DIRECTORIES include test/common)

        ##### GLOG Tests #####
        set(GLOG_TESTS "test_glog")

        add_executable(${GLOG_TESTS} test/test_entry_point.cpp
                test/glog/test_glog_basic.cc
                test/glog/test_glog_every_n.cc
                test/glog/test_glog_first_n.cc
                test/glog/test_glog_log_string.cc
                test/glog/test_glog_rosprintf.cc
                test/glog/test_glog_vlog.cc)

        target_include_directories(${GLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/glog )
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

        target_include_directories(${LPP_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/lpp )
        target_link_libraries(${LPP_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${LPP_TESTS} PRIVATE MODE_LPP)


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

        target_include_directories(${DEFAULT_TESTS} PRIVATE  ${LPP_INCLUDE_DIRECTORIES} test/default)
        target_link_libraries(${DEFAULT_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${DEFAULT_TESTS} PRIVATE MODE_DEFAULT)
    endif ()
endif ()


# Valid modes are: MODE_GLOG MODE_ROSLOG MODE_LPP MODE_DEFAULT
