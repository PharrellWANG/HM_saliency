cmake_minimum_required(VERSION 2.8)
project(TAppEncoder)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../../Lib/
)

set(SOURCE_FILES
    encmain.cpp
    TAppEncCfg.cpp
    TAppEncCfg.h
    TAppEncTop.cpp
    TAppEncTop.h)

add_executable(TAppEncoder ${SOURCE_FILES})
target_link_libraries(TAppEncoder TAppCommon TLibCommon TLibEncoder TLibDecoder TLibVideoIO)
