cmake_minimum_required(VERSION 3.16)
project(Log++)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake")

include_directories(Include)

find_package(glog)
find_package(catkin REQUIRED COMPONENTS roscpp)

add_executable(Log++ src/main.cpp)


if (catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})
    target_link_libraries(Log++ ${catkin_LIBRARIES})
    add_compile_definitions(ROSLOG_SUPPORTED)
endif ()

if (GLOG_FOUND)
    add_compile_definitions(GLOG_SUPPORTED)
    target_link_libraries(Log++ glog)
endif ()


if (GLOG_FOUND)
    if (catkin_FOUND)
        add_executable(Logtest test/test_entry_point.cpp
                Include/log++.h
                test/glog/test_glog.cc

                )
        target_include_directories(Logtest PRIVATE test)

        target_link_libraries(Logtest glog ${catkin_LIBRARIES})
        target_link_libraries(Logtest gtest)
    endif ()
endif ()


# Valid modes are: MODE_GLOG MODE_ROSLOG MODE_LPP MODE_DEFAULT
target_compile_definitions(Log++ PRIVATE MODE_GLOG)
