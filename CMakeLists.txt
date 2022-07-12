cmake_minimum_required(VERSION 3.2)

project(elevator)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall")

add_executable(elevator src/main.cpp)
