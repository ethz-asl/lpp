cmake_minimum_required(VERSION 3.16)
project(Log++)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

include_directories(include)

find_package(glog)
find_package(catkin REQUIRED COMPONENTS roscpp)

add_executable(Log++ src/main.cpp)

if (catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})
    target_link_libraries(Log++ ${catkin_LIBRARIES})
endif ()

if (GLOG_FOUND)
    target_link_libraries(Log++ glog)
endif ()

# All logging frameworks are required for unittests
if (GLOG_FOUND)
    if (catkin_FOUND)

        message("-- Found glog and catkin: Log++ tests will be built")
        ##### Common includes #####
        set(LPP_INCLUDE_DIRECTORIES include test/common)

        ##### GLOG Tests #####
        set(GLOG_TESTS "test_glog")

        add_executable(${GLOG_TESTS} test/test_entry_point.cpp
                test/glog/test_basic_glog.cc
                )

        target_include_directories(${GLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/glog )
        target_link_libraries(${GLOG_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${GLOG_TESTS} PRIVATE MODE_GLOG)


        ##### Log++ Tests #####
        set(LPP_TESTS "test_lpp")
        add_executable(${LPP_TESTS} test/test_entry_point.cpp
                test/lpp/test_basic_lpp.cc
                test/lpp/test_first_n.cc
                test/lpp/test_every_n.cc)

        target_include_directories(${LPP_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/lpp )
        target_link_libraries(${LPP_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${LPP_TESTS} PRIVATE MODE_LPP)


        ##### Roslog Tests #####
        set(ROSLOG_TESTS "test_roslog")
        add_executable(${ROSLOG_TESTS} test/test_entry_point.cpp
                test/roslog/test_basic_roslog.cc
                )

        target_include_directories(${ROSLOG_TESTS} PRIVATE ${LPP_INCLUDE_DIRECTORIES} test/roslog)
        target_link_libraries(${ROSLOG_TESTS} glog gtest ${catkin_LIBRARIES})
        target_compile_definitions(${ROSLOG_TESTS} PRIVATE MODE_ROSLOG)

    endif ()
endif ()


# Valid modes are: MODE_GLOG MODE_ROSLOG MODE_LPP MODE_DEFAULT
