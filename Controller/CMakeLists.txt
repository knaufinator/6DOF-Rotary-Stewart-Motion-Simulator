cmake_minimum_required(VERSION 3.10)
project(StewartPlatform VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to build for testing/Python integration
option(BUILD_SHARED_LIB "Build shared library for Python integration" ON)

# Include directories
include_directories(include)

# Source files for the inverse kinematics
set(IK_SOURCES
    src/InverseKinematics.cpp
)

# Create shared library for Python integration
if(BUILD_SHARED_LIB)
    add_library(inverse_kinematics SHARED ${IK_SOURCES})
    
    # Set output directory for the shared library
    set_target_properties(inverse_kinematics PROPERTIES 
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    
    # Set up correct naming for different platforms
    if(WIN32)
        set_target_properties(inverse_kinematics PROPERTIES SUFFIX ".dll")
    elseif(APPLE)
        set_target_properties(inverse_kinematics PROPERTIES SUFFIX ".dylib")
    else()
        set_target_properties(inverse_kinematics PROPERTIES SUFFIX ".so")
    endif()
    
    # Install rules
    install(TARGETS inverse_kinematics
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()
