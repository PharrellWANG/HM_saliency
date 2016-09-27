cmake_minimum_required(VERSION 2.8)
project(TAppCommon)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../
)

set(SOURCE_FILES
    program_options_lite.cpp
    program_options_lite.h)

ADD_LIBRARY(TAppCommon STATIC ${SOURCE_FILES})