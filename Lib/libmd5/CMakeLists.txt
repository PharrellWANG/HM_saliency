cmake_minimum_required(VERSION 2.8)
project(libmd5)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    libmd5.c
    libmd5.h
    MD5.h)

ADD_LIBRARY(libmd5 STATIC ${SOURCE_FILES})