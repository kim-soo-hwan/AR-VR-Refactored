cmake_minimum_required(VERSION 3.12)
project(Orange_Triangle)

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
add_executable(08_Orange_Triangle ${SRCS})

# linking
target_link_libraries(08_Orange_Triangle glfw glad)

# shader
#configure_file(default.vert ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#configure_file(orange.frag  ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
add_custom_command(TARGET 08_Orange_Triangle POST_BUILD
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/default.vert" $<TARGET_FILE_DIR:08_Orange_Triangle>
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/orange.frag"  $<TARGET_FILE_DIR:08_Orange_Triangle>
                   COMMENT "Copying dependent shaders"
)