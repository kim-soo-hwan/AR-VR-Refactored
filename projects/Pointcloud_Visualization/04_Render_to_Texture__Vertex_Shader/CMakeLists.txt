cmake_minimum_required(VERSION 3.0)

# project
project(Render_to_Texture__Vertex_Shader)

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
#find_package(PCL 1.3 REQUIRED)
find_package(PCL CONFIG REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# executable
add_executable(94_Render_to_Texture__Vertex_Shader_01 "main_01.cpp")
add_executable(94_Render_to_Texture__Vertex_Shader_02 "main_02.cpp")

# linking
target_link_libraries(94_Render_to_Texture__Vertex_Shader_01 ${PCL_LIBRARIES} glfw glad swan)
target_link_libraries(94_Render_to_Texture__Vertex_Shader_02 ${PCL_LIBRARIES} glfw glad swan)

# point cloud data
list(APPEND DATA_01 "MobilTech_01/000000001.pcd" "MobilTech_01/0.jpg" "MobilTech_01/1.jpg" "MobilTech_01/2.jpg" "MobilTech_01/3.jpg" "MobilTech_01/4.jpg")
add_custom_command(TARGET 94_Render_to_Texture__Vertex_Shader_01
    POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory $<TARGET_FILE_DIR:94_Render_to_Texture__Vertex_Shader_01>/MobilTech_01
    COMMENT "Creating a data directory"
)

add_custom_command(TARGET 94_Render_to_Texture__Vertex_Shader_01
    POST_BUILD
    WORKING_DIRECTORY ${DATA_DIR}
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${DATA_01} $<TARGET_FILE_DIR:94_Render_to_Texture__Vertex_Shader_01>/MobilTech_01
    COMMENT "Copying dependent point cloud data"
)

list(APPEND DATA_02 "MobilTech_02/000000.pcd" "MobilTech_02/0.jpg" "MobilTech_02/1.jpg" "MobilTech_02/2.jpg" "MobilTech_02/3.jpg" "MobilTech_02/4.jpg")
add_custom_command(TARGET 94_Render_to_Texture__Vertex_Shader_02
    POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory $<TARGET_FILE_DIR:94_Render_to_Texture__Vertex_Shader_02>/MobilTech_02
    COMMENT "Creating a data directory"
)

add_custom_command(TARGET 94_Render_to_Texture__Vertex_Shader_02
    POST_BUILD
    WORKING_DIRECTORY ${DATA_DIR}
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${DATA_02} $<TARGET_FILE_DIR:94_Render_to_Texture__Vertex_Shader_02>/MobilTech_02
    COMMENT "Copying dependent point cloud data"
)

# shaders
list(APPEND SHADERS "multiple_model_view_distorted_projections_pointSize.vs"
                    "projected_color.fs")
add_custom_command(TARGET 94_Render_to_Texture__Vertex_Shader_01
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:94_Render_to_Texture__Vertex_Shader_01>
    COMMENT "Copying dependent shaders"
)
add_custom_command(TARGET 94_Render_to_Texture__Vertex_Shader_02
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:94_Render_to_Texture__Vertex_Shader_02>
    COMMENT "Copying dependent shaders"
)