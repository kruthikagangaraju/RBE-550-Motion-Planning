cmake_minimum_required(VERSION 3.10)

project(Project4)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(car car.cpp)
target_link_libraries(car ${OMPL_LIBRARIES})

add_executable(pendulum pendulum.cpp)
target_link_libraries(pendulum ${OMPL_LIBRARIES})

