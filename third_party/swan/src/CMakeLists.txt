cmake_minimum_required(VERSION 3.12)
project(Swan)

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
list(APPEND SRCS "window.cpp" 
                 "mesh.cpp" 
                 "utility.cpp" 
                 "shader.cpp" 
                 "texture.cpp" 
                 "model.cpp"
                 "scene.cpp"
                 "camera.cpp"
                 "axes.cpp"
                 "rigid.cpp")

# library
add_library(swan ${SRCS})