add_library(dq1 STATIC Modeling.cpp)

set_property(TARGET dq1 PROPERTY VERSION ${PACKAGE_VERSION})
set_property(TARGET dq1 PROPERTY SOVERSION ${PACKAGE_SO_VERSION})

target_include_directories(dq1
                           PUBLIC
                            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/dq1>
                            $<INSTALL_INTERFACE:include/dq1>
                           )
# state that SqrtLibrary need PIC when the default is shared libraries
set_target_properties(dq1 PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
                    )

find_package(
    qpOASES REQUIRED
    nlohmann_json 3.11.3 REQUIRED
)
target_link_libraries(dq1 PUBLIC dq1_compiler_flags)
target_link_libraries(dq1 PRIVATE
    qpOASES
)

set(installable_libs dq1 dq1_compiler_flags)
install(TARGETS ${installable_libs}
    EXPORT dq1Targets
    DESTINATION lib
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp")

# Examples
message(STATUS "dq1_BUILD_EXAMPLES is set to ${dq1_BUILD_EXAMPLES}")

if(dq1_BUILD_EXAMPLES)
    # Compile dq1 examples
    set(EXAMPLE_NAMES
        example1
    )

    foreach(EXAMPLE ${EXAMPLE_NAMES})
        add_executable(${EXAMPLE} ${EXAMPLE}.cpp)
        target_link_libraries(${EXAMPLE} dq1)
    endforeach()
endif()