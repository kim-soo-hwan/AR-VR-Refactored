cmake_minimum_required(VERSION 3.12)
project(Red_Triangle)

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
list(APPEND SRCS "main.cpp" "window.cpp" "mesh.cpp" "shader.cpp")

# executable
add_executable(10_Red_Triangle ${SRCS})

# linking
target_link_libraries(10_Red_Triangle glfw glad)

# shaders
list(APPEND SHADERS "${CMAKE_CURRENT_SOURCE_DIR}/pass_red_color.vert"
                    "${CMAKE_CURRENT_SOURCE_DIR}/default.frag"
)
add_custom_command(TARGET 10_Red_Triangle POST_BUILD
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:10_Red_Triangle>
                   COMMENT "Copying dependent shaders"
)