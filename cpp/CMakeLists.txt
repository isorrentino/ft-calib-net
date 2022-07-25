cmake_minimum_required(VERSION 3.16)
project(test_keras_model)

find_package(frugally-deep REQUIRED)

add_executable(test_keras_model main.cpp)
target_link_libraries(test_keras_model frugally-deep::fdeep)
