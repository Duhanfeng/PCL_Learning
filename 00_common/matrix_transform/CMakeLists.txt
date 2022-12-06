cmake_minimum_required(VERSION 3.24)
project(matrix_transform)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

#设置生成时操作
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/data/bunny/data ${PROJECT_BINARY_DIR}/data
        )
