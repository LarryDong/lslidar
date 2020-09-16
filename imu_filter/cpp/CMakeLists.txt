cmake_minimum_required(VERSION 2.8)
project(imu_filter)
find_package(Threads)
find_package(Boost)
set(CMAKE_CXX_STANDARD 11)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)


add_executable(imu_filter 
    src/KVH1750.cpp
    src/main.cpp 
    src/IMU_filter.cpp
)


target_link_libraries(
    imu_filter
    -lboost_system
    -lpthread
)