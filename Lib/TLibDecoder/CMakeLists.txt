cmake_minimum_required(VERSION 2.8)
project(TLibDecoder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../
)

set(SOURCE_FILES
    AnnexBread.cpp
    AnnexBread.h
    NALread.cpp
    NALread.h
    SEIread.cpp
    SEIread.h
    SyntaxElementParser.cpp
    SyntaxElementParser.h
    TDecBinCoder.h
    TDecBinCoderCABAC.cpp
    TDecBinCoderCABAC.h
    TDecCAVLC.cpp
    TDecCAVLC.h
    TDecCu.cpp
    TDecCu.h
    TDecEntropy.cpp
    TDecEntropy.h
    TDecGop.cpp
    TDecGop.h
    TDecSbac.cpp
    TDecSbac.h
    TDecSlice.cpp
    TDecSlice.h
    TDecTop.cpp
    TDecTop.h)

ADD_LIBRARY(TLibDecoder STATIC ${SOURCE_FILES})