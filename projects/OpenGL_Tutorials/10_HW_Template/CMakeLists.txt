cmake_minimum_required(VERSION 3.12)
project(HW_01)

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
list(APPEND SRCS "window.cpp" "mesh.cpp" "shader.cpp")

# executable
add_executable(HW_01_01 "01_main.cpp" ${SRCS})
add_executable(HW_01_02 "02_main.cpp" ${SRCS})
add_executable(HW_01_03 "03_main.cpp" ${SRCS})

# linking
target_link_libraries(HW_01_01 glfw glad)
target_link_libraries(HW_01_02 glfw glad)
target_link_libraries(HW_01_03 glfw glad)

# shaders
list(APPEND SHADERS "pass_red_color.vert"
                    "default.frag"
)
add_custom_command(TARGET HW_01_01 POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:HW_01_01>
                   COMMENT "Copying dependent shaders"
)
add_custom_command(TARGET HW_01_02 POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:HW_01_02>
                   COMMENT "Copying dependent shaders"
)
add_custom_command(TARGET HW_01_03 POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${SHADERS} $<TARGET_FILE_DIR:HW_01_03>
                   COMMENT "Copying dependent shaders"
)