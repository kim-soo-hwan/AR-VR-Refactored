cmake_minimum_required(VERSION 3.12)
project(Swan)

# standard
set(CMAKE_CXX_STANDARD 11)

# OS
if(WIN32)
    add_definitions(-D__WINDOWS__)
elseif(UNIX)
    if(APPLE)
        add_definitions(-D__APPLE__)
    endif()
endif()

# source files
list(APPEND SRCS "window.cpp" "mesh.cpp" "utility.cpp")

# library
add_library(swan ${SRCS})