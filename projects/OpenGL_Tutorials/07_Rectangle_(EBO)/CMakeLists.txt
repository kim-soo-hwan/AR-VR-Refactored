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

# source files
list(APPEND SRCS "window.cpp" "mesh.cpp")

# executable
add_executable(07_01_Rectangle          "01_main_rectangle.cpp"           ${SRCS})
add_executable(07_02_Rectangle_w_Shader "02_main_rectangle_w_shader.cpp"  ${SRCS})

# linking
target_link_libraries(07_01_Rectangle           glfw glad)
target_link_libraries(07_02_Rectangle_w_Shader  glfw glad)