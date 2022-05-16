cmake_minimum_required(VERSION 3.12)
project(Window_GLAD)

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
add_executable(02_Window_GLAD ${SRCS})

# linking
target_link_libraries(02_Window_GLAD glfw glad)