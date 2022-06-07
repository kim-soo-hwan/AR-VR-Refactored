cmake_minimum_required(VERSION 3.0)

# vcpkg
# https://vcpkg.readthedocs.io/en/latest/users/integration/#using-an-environment-variable-instead-of-a-command-line-option
# https://vcpkg.io/en/getting-started.html
#set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")
#if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
#  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
#      CACHE STRING "")
#endif()

# project
project(PCL_CloudViewer)

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

# executable
add_executable(91_PCL_CloudViewer "main.cpp")

# linking
target_link_libraries(91_PCL_CloudViewer ${PCL_LIBRARIES})

# point cloud data
list(APPEND PCD "MobilTech_01/000000001.pcd")
add_custom_command(TARGET 91_PCL_CloudViewer
    POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory $<TARGET_FILE_DIR:91_PCL_CloudViewer>/MobilTech_01
    COMMENT "Creating a data directory"
)
add_custom_command(
    TARGET 91_PCL_CloudViewer POST_BUILD
        WORKING_DIRECTORY ${DATA_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${PCD} $<TARGET_FILE_DIR:91_PCL_CloudViewer>/MobilTech_01
        COMMENT "Copying dependent point cloud data"
)