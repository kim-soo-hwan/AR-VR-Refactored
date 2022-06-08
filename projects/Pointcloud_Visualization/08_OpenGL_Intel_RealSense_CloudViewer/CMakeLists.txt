cmake_minimum_required(VERSION 3.0)

# project
project(OpenGL_Intel_RealSense_CloudViewer)

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

# Intel RealSense
#set(realsense2_DIR "D:\\vcpkg\\installed\\x86-windows\\share\\realsense2")
#set(CMAKE_TOOLCHAIN_FILE "D:\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
find_package(realsense2 CONFIG REQUIRED)

# executable
add_executable(98_OpenGL_Intel_RealSense_CloudViewer "main.cpp")

# linking
#target_link_libraries(98_OpenGL_Intel_RealSense_CloudViewer PRIVATE glfw glad realsense2::fw realsense2::realsense2 realsense2::realsense-file)
target_link_libraries(98_OpenGL_Intel_RealSense_CloudViewer PRIVATE glfw glad swan realsense2::realsense2 realsense2::realsense-file)

# shaders
list(APPEND SHADERS "model_view_projection_pointSize_PCL_XYZRGB.vs" 
                    "vertex_color.fs")

add_custom_command(
    TARGET 98_OpenGL_Intel_RealSense_CloudViewer POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:98_OpenGL_Intel_RealSense_CloudViewer>
        COMMENT "Copying dependent shaders"
)