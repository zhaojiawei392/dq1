cmake_minimum_required(VERSION 3.12)
project(DualQuaternion)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(test1 
    test1.cpp
)

add_executable(test2 
    test2.cpp
)