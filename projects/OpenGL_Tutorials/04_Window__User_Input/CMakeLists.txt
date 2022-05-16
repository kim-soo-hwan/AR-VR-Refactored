cmake_minimum_required(VERSION 3.12)
project(Window_User_Input)

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
add_executable(04_Window_User_Input "main.cpp")

# linking
target_link_libraries(04_Window_User_Input glfw glad swan)