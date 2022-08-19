cmake_minimum_required(VERSION 3.16)
project(Log++)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake")

find_package(glog)
find_package(catkin REQUIRED COMPONENTS roscpp)

add_executable(Log++ main.cpp)
add_executable(Logtest test.cpp)

if (catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})
    target_link_libraries(Log++ ${catkin_LIBRARIES})
    add_compile_definitions(ROSLOG_SUPPORTED)
endif()

if(GLOG_FOUND)
    add_compile_definitions(GLOG_SUPPORTED)
    target_link_libraries(Log++ glog)
endif()

