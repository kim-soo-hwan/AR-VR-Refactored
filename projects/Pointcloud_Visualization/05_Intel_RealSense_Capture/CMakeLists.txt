cmake_minimum_required(VERSION 3.0)

# project
project(IntelRealSenseCapture)

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
add_executable(95_Intel_RealSense_Capture "main.cpp")

# linking
target_link_libraries(95_Intel_RealSense_Capture PRIVATE glfw glad realsense2::fw realsense2::realsense2 realsense2::realsense-file)