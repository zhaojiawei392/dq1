cmake_minimum_required(VERSION 3.12)
project(dq1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add an INTERFACE library target for the header-only library
add_library(dq1 INTERFACE)

# Specify the include directories for the library
target_include_directories(dq1 INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Specify installation instructions for the header files
install(DIRECTORY include/ DESTINATION include/dq1)

# Install CMake package configuration files
install(TARGETS dq1 EXPORT dq1Config)

# Export the configuration for find_package
install(EXPORT dq1Config
    NAMESPACE dq1::
    DESTINATION lib/cmake/dq1
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# add_executable(test1 
#     test1.cpp
# )

# add_executable(test2 
#     test2.cpp
# )

add_executable(test3 
    test3.cpp
)