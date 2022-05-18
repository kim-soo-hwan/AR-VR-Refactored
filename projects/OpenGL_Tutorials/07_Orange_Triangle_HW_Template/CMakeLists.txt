cmake_minimum_required(VERSION 3.12)
project(Orange_Triangle)

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
add_executable(07_Orange_Triangle_HW_01    "01_main_two_orange_triangles.cpp")
add_executable(07_Orange_Triangle_HW_02    "02_main_two_individual_orange_triangles.cpp")
add_executable(07_Orange_Triangle_HW_03    "03_main_one_orange_and_one_yellow_triangles.cpp")

# linking
target_link_libraries(07_Orange_Triangle_HW_01    glfw glad swan)
target_link_libraries(07_Orange_Triangle_HW_02    glfw glad swan)
target_link_libraries(07_Orange_Triangle_HW_03    glfw glad swan)

# shaders
list(APPEND SHADERS_01 "default.vs" "orange.fs")
list(APPEND SHADERS_02 "default.vs" "orange.fs")
list(APPEND SHADERS_03 "default.vs" "orange.fs" "yellow.fs")

add_custom_command(
    TARGET 07_Orange_Triangle_HW_01 POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS_01} $<TARGET_FILE_DIR:07_Orange_Triangle_HW_01>
        COMMENT "Copying dependent shaders"
)
add_custom_command(
    TARGET 07_Orange_Triangle_HW_02 POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS_02} $<TARGET_FILE_DIR:07_Orange_Triangle_HW_02>
        COMMENT "Copying dependent shaders"
)
add_custom_command(
    TARGET 07_Orange_Triangle_HW_03 POST_BUILD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS_03} $<TARGET_FILE_DIR:07_Orange_Triangle_HW_03>
        COMMENT "Copying dependent shaders"
)