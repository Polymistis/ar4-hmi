cmake_minimum_required(VERSION 3.15)
project(robot_kinematics)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(pybind11 REQUIRED)

pybind11_add_module(robot_kinematics bindings.cpp)
target_compile_definitions(robot_kinematics PRIVATE $<$<CONFIG:Release>:NDEBUG>)

if(MSVC)
    target_compile_options(
        robot_kinematics
        PRIVATE
        /W4
        /WX
        /permissive-
        /EHsc
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:Release>:/Ob2>
    )
    target_link_options(
        robot_kinematics
        PRIVATE
        /INCREMENTAL:NO
        $<$<CONFIG:Release>:/OPT:REF>
        $<$<CONFIG:Release>:/OPT:ICF>
    )
else()
    target_compile_options(
        robot_kinematics
        PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        $<$<CONFIG:Release>:-O3>
    )
endif()
