cmake_minimum_required(VERSION 3.12)
project(Window_User_Input)

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
list(APPEND SRCS "main.cpp" "window.cpp")

# executable
add_executable(04_Window_User_Input ${SRCS})

# linking
target_link_libraries(04_Window_User_Input glfw glad)