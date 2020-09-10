cmake_minimum_required(VERSION 3.17)
project(test_IMU)
find_package(Threads)
find_package(Boost)
set(CMAKE_CXX_STANDARD 11)

include_directories(
        ${Boost_INCLUDE_DIRS}
)


add_executable(test_IMU main.cpp)
target_link_libraries(
        test_IMU
        -lboost_system
        -lpthread
)