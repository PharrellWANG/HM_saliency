cmake_minimum_required(VERSION 2.8)
project(TLibVideoIO)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../
)

set(SOURCE_FILES
    TVideoIOYuv.cpp
    TVideoIOYuv.h)

ADD_LIBRARY(TLibVideoIO STATIC ${SOURCE_FILES})