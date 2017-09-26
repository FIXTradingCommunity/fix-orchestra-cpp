project(score-dsl-cpp)
cmake_minimum_required(VERSION 2.8)

aux_source_directory(src SRC_LIST)
include_directories(src)

add_executable(${PROJECT_NAME} ${SRC_LIST})
