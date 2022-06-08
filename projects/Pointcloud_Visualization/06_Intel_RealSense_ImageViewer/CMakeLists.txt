cmake_minimum_required(VERSION 3.0)

# project
project(Intel_RealSense_ImageViewer)

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

# Intel RealSense
#set(realsense2_DIR "D:\\vcpkg\\installed\\x86-windows\\share\\realsense2")
#set(CMAKE_TOOLCHAIN_FILE "D:\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
find_package(realsense2 CONFIG REQUIRED)

# executable
add_executable(96_Intel_RealSense_ImageViewer "main.cpp")

# linking
#target_link_libraries(96_Intel_RealSense_ImageViewer PRIVATE glfw glad realsense2::fw realsense2::realsense2 realsense2::realsense-file)
target_link_libraries(96_Intel_RealSense_ImageViewer PRIVATE glfw glad swan realsense2::realsense2 realsense2::realsense-file)

# shaders 
list(APPEND SHADERS "pass_texCoords.vs" "single_texture.fs")
add_custom_command(
    TARGET 96_Intel_RealSense_ImageViewer POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:96_Intel_RealSense_ImageViewer>
        COMMENT "Copying dependent shaders"
)