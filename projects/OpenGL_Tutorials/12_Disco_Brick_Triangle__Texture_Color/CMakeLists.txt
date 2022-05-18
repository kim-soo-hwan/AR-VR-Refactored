cmake_minimum_required(VERSION 3.12)
project(Disco_Brick_Triangle)

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
add_executable(12_Disco_Brick_Triangle "main.cpp")

# linking
target_link_libraries(12_Disco_Brick_Triangle glfw glad swan)

# shaders 
list(APPEND SHADERS "pass_color_texCoords.vs" "texture_and_color.fs")
add_custom_command(
    TARGET 12_Disco_Brick_Triangle POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:12_Disco_Brick_Triangle>
        COMMENT "Copying dependent shaders"
)

# images
list(APPEND IMAGES "wall.jpg")
add_custom_command(
    TARGET 12_Disco_Brick_Triangle POST_BUILD
        WORKING_DIRECTORY ${DATA_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${IMAGES} $<TARGET_FILE_DIR:12_Disco_Brick_Triangle>
        COMMENT "Copying dependent images"
)