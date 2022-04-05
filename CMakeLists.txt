cmake_minimum_required(VERSION 3.0.0)
project(Top)

# GLFW 3.3.6
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/glfw-3.3.6)

# GLAD (OpenGL Core 4.1)
set (GLAD_DIR ${CMAKE_SOURCE_DIR}/third_party/glad)
include_directories(${GLAD_DIR}/include)
add_library(glad ${GLAD_DIR}/src/glad.c)

# STB
include_directories(${CMAKE_SOURCE_DIR}/third_party/stb)

# GLM
include_directories(${CMAKE_SOURCE_DIR}/third_party/glm-0.9.9.8)

# Data
set (DATA_DIR ${CMAKE_SOURCE_DIR}/data)

# Projects
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/01_Hello_World_(GLFW)")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/02_Window_(GLAD)")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/03_Window_(Refactoring)")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/04_Window_(User_Input)")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/05_Triangles_(VAO_VBO_VAP)")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/07_Rectangle_(EBO)")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/08_Orange_Triangle_(Vertex_Fragment_Shaders)")