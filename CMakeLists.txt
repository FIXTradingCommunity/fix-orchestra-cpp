project(score-dsl-cpp)
cmake_minimum_required(VERSION 2.8)

aux_source_directory(src SRC_LIST)
include_directories(src)

set(TARGET scoretool)
add_executable(${TARGET} ${SRC_LIST})

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${Boost_LIBRARIES})
