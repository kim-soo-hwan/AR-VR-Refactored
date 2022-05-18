cmake_minimum_required(VERSION 3.0.0)
project(Top)

# GLFW 3.3.6
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
include_directories(${CMAKE_SOURCE_DIR}/third_party/glfw-3.3.6/include)
add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/glfw-3.3.6)

# GLAD (OpenGL Core 4.1)
set (GLAD_DIR ${CMAKE_SOURCE_DIR}/third_party/glad)
include_directories(${GLAD_DIR}/include)
add_library(glad ${GLAD_DIR}/src/glad.c)

# STB
include_directories(${CMAKE_SOURCE_DIR}/third_party/stb)

# GLM
include_directories(${CMAKE_SOURCE_DIR}/third_party/glm-0.9.9.8)

# Swan
include_directories(${CMAKE_SOURCE_DIR}/third_party/swan/include)
add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/swan/src)

# Data
set (DATA_DIR ${CMAKE_SOURCE_DIR}/data)

# Projects
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/01_Hello_World__GLFW")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/02_Window__GLAD")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/03_Window__Refactoring")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/04_Window__User_Input")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/05_Triangles__VAO_VBO_VAP")
#add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/05_Triangles_HW_Template")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/06_Rectangle__EBO")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/07_Orange_Triangle__Vertex_Fragment_Shaders")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/07_Orange_Triangle_HW_Template")
#add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/07_Orange_Triangle_HW_Answer")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/08_Red_Triangle__GLSL_In_Out")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/09_Greenish_Triangle__GLSL_Uniform")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/09_Greenish_Triangle_HW_Template")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/10_Colorful_Triangle__GLSL_layout_location")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/11_Brick_Triangle__Texture")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/12_Disco_Brick_Triangle__Texture_Color")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/13_Smiling_Wooden_Rectangle__Texture_Units")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/14_Transformed_Rectangle_in_2D__Model")
add_subdirectory("${CMAKE_SOURCE_DIR}/projects/OpenGL_Tutorials/15_Rotating_Rectangle_in_2D__Model_Uniform")