cmake_minimum_required(VERSION 3.12)
project(Triangles)

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
add_executable(05_01_Triangle           "01_main_triangle.cpp"           ${SRCS})
add_executable(05_02_Triangle_w_Shader  "02_main_triangle_w_shader.cpp"  ${SRCS})
add_executable(05_03_Rectangle          "03_main_rectangle.cpp"          ${SRCS})
add_executable(05_04_Rectangle_w_Shader "04_main_rectangle_w_shader.cpp" ${SRCS})

# linking
target_link_libraries(05_01_Triangle           glfw glad)
target_link_libraries(05_02_Triangle_w_Shader  glfw glad)
target_link_libraries(05_03_Rectangle          glfw glad)
target_link_libraries(05_04_Rectangle_w_Shader glfw glad)