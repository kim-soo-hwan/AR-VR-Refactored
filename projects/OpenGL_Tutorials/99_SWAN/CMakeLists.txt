cmake_minimum_required(VERSION 3.12)
project(Translating_Camera_HW)

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
list(APPEND SRCS "main.cpp" "window.cpp" "mesh.cpp" "shader.cpp" "texture.cpp" "camera.cpp" "model.cpp")

# executable
add_executable(27_Translating_Camera_HW ${SRCS})

# linking
target_link_libraries(27_Translating_Camera_HW glfw glad)

# shaders 
list(APPEND SHADERS "model_view_projection_pass_texCoords.vert" "two_textures.frag")
add_custom_command(
    TARGET 27_Translating_Camera_HW POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:27_Translating_Camera_HW>
        COMMENT "Copying dependent shaders and images"
)

# images
list(APPEND IMAGES "container.jpg" "awesomeface.png")
add_custom_command(
    TARGET 27_Translating_Camera_HW POST_BUILD
        WORKING_DIRECTORY ${DATA_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${IMAGES} $<TARGET_FILE_DIR:27_Translating_Camera_HW>
        COMMENT "Copying dependent shaders and images"
)