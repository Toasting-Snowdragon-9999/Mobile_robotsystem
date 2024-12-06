cmake_minimum_required(VERSION 3.5)

project(Test_methodes_DSP_all LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Test_methodes_DSP_all main.cpp dft.cpp dft.h goertzel.h goertzel.cpp)
