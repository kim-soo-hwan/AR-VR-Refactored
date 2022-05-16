cmake_minimum_required(VERSION 3.12)
project(Hello_World)

# standard
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# OS
if(WIN32)
    add_definitions(-D__WINDOWS__)
elseif(UNIX)
    if(APPLE)
        add_definitions(-D__APPLE__)
    endif()
endif()

# source files
list(APPEND SRCS "main.cpp")

# executable
add_executable(01_Hello_World ${SRCS})

# linking
target_link_libraries(01_Hello_World glfw)