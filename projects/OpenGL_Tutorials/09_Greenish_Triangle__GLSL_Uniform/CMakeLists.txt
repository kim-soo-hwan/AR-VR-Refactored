cmake_minimum_required(VERSION 3.12)
project(Greenish_Triangle)

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
add_executable(09_Greenish_Triangle "main.cpp")

# linking
target_link_libraries(09_Greenish_Triangle glfw glad swan)

# shaders
list(APPEND SHADERS "default.vs" "uniform_color.fs")

add_custom_command(
    TARGET 09_Greenish_Triangle POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:09_Greenish_Triangle>
        COMMENT "Copying dependent shaders"
)