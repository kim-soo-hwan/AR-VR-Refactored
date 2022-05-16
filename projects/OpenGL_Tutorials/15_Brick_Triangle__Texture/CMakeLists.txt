cmake_minimum_required(VERSION 3.12)
project(Brick_Triangle)

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
add_executable(15_Brick_Triangle_01 "01_main.cpp")
add_executable(15_Brick_Triangle_02 "02_main.cpp")

# linking
target_link_libraries(15_Brick_Triangle_01 glfw glad swan)
target_link_libraries(15_Brick_Triangle_02 glfw glad swan)

# shaders 
list(APPEND SHADERS "pass_texCoords.vs" "texture.fs")
add_custom_command(
    TARGET 15_Brick_Triangle_01 POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:15_Brick_Triangle_01>
        COMMENT "Copying dependent shaders"
)
add_custom_command(
    TARGET 15_Brick_Triangle_02 POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:15_Brick_Triangle_02>
        COMMENT "Copying dependent shaders"
)

# images
list(APPEND IMAGES "wall.jpg")
add_custom_command(
    TARGET 15_Brick_Triangle_01 POST_BUILD
        WORKING_DIRECTORY ${DATA_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${IMAGES} $<TARGET_FILE_DIR:15_Brick_Triangle_01>
        COMMENT "Copying dependent images"
)
add_custom_command(
    TARGET 15_Brick_Triangle_02 POST_BUILD
        WORKING_DIRECTORY ${DATA_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${IMAGES} $<TARGET_FILE_DIR:15_Brick_Triangle_02>
        COMMENT "Copying dependent images"
)