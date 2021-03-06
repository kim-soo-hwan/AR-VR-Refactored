cmake_minimum_required(VERSION 3.12)
project(HW_01)

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
add_executable(09_Geenish_Triangle_HW_01 "01_main.cpp")
add_executable(09_Geenish_Triangle_HW_02 "02_main.cpp")
add_executable(09_Geenish_Triangle_HW_03 "03_main.cpp")

# linking
target_link_libraries(09_Geenish_Triangle_HW_01 glfw glad swan)
target_link_libraries(09_Geenish_Triangle_HW_02 glfw glad swan)
target_link_libraries(09_Geenish_Triangle_HW_03 glfw glad swan)

# shaders
list(APPEND SHADERS "pass_red_color.vs" "default.fs")

add_custom_command(TARGET 09_Geenish_Triangle_HW_01 POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:09_Geenish_Triangle_HW_01>
                   COMMENT "Copying dependent shaders"
)
add_custom_command(TARGET 09_Geenish_Triangle_HW_02 POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:09_Geenish_Triangle_HW_02>
                   COMMENT "Copying dependent shaders"
)
add_custom_command(TARGET 09_Geenish_Triangle_HW_03 POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:09_Geenish_Triangle_HW_03>
                   COMMENT "Copying dependent shaders"
)