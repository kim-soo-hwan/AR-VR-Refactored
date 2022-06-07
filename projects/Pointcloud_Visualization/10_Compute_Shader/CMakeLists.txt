cmake_minimum_required(VERSION 3.0)

# project
project(Compute_Shader)

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

# PCL
#find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# executable
add_executable(94_Compute_Shader "main.cpp")

# linking
target_link_libraries(94_Compute_Shader ${PCL_LIBRARIES} glfw glad swan)

# shaders
list(APPEND SHADERS "model_view_projection_colormap_jet.vs"
                    "vertex_color.fs")
add_custom_command(
    TARGET 94_Compute_Shader POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:94_Compute_Shader>
        COMMENT "Copying dependent shaders"
)