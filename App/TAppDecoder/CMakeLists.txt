cmake_minimum_required(VERSION 2.8)
project(TAppDecoder)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../../Lib/
)

set(SOURCE_FILES
    decmain.cpp
    TAppDecCfg.cpp
    TAppDecCfg.h
    TAppDecTop.cpp
    TAppDecTop.h)

add_executable(TAppDecoder ${SOURCE_FILES})
target_link_libraries(TAppDecoder TAppCommon TLibCommon TLibEncoder TLibDecoder TLibVideoIO)
