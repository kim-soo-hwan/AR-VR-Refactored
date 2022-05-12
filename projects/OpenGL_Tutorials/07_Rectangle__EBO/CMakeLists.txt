cmake_minimum_required(VERSION 3.12)
project(Rectangle)

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

# executable
add_executable(07_Rectangle "main_rectangle.cpp")

# linking
target_link_libraries(07_Rectangle glfw glad swan)