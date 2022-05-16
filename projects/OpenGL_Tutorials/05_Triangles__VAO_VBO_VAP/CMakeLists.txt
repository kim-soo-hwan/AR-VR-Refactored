cmake_minimum_required(VERSION 3.12)
project(Triangles)

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

# executable
add_executable(05_01_Triangle  "01_main_triangle.cpp" )
add_executable(05_02_Rectangle "02_main_rectangle.cpp")

# linking
target_link_libraries(05_01_Triangle  glfw glad swan)
target_link_libraries(05_02_Rectangle glfw glad swan)