cmake_minimum_required(VERSION 3.12)
project(Many_Cubes_3D)

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
add_executable(19_Many_Cubes_3D_01 "01_main.cpp")
add_executable(19_Many_Cubes_3D_02 "02_main.cpp")

# linking
target_link_libraries(19_Many_Cubes_3D_01 glfw glad swan)
target_link_libraries(19_Many_Cubes_3D_02 glfw glad swan)

# shaders 
list(APPEND SHADERS "model_view_projection_pass_texCoords.vs" "two_textures.fs")
add_custom_command(
    TARGET 19_Many_Cubes_3D_01 POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:19_Many_Cubes_3D_01>
        COMMENT "Copying dependent shaders"
)
add_custom_command(
    TARGET 19_Many_Cubes_3D_02 POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:19_Many_Cubes_3D_02>
        COMMENT "Copying dependent shaders"
)

# images
list(APPEND IMAGES "container.jpg" "awesomeface.png")
add_custom_command(
    TARGET 19_Many_Cubes_3D_01 POST_BUILD
        WORKING_DIRECTORY ${DATA_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${IMAGES} $<TARGET_FILE_DIR:19_Many_Cubes_3D_01>
        COMMENT "Copying dependent images"
)
add_custom_command(
    TARGET 19_Many_Cubes_3D_02 POST_BUILD
        WORKING_DIRECTORY ${DATA_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${IMAGES} $<TARGET_FILE_DIR:19_Many_Cubes_3D_02>
        COMMENT "Copying dependent images"
)