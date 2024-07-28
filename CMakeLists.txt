cmake_minimum_required(VERSION 3.12)
project(DualQuaternion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(test1 
    test1.cpp
    src/Quaternion.cpp
)

add_executable(test2 
    test2.cpp
    src/Quaternion.cpp
)