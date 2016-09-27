cmake_minimum_required(VERSION 2.8)
project(TLibEncoder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../
)

set(SOURCE_FILES
    AnnexBwrite.h
    NALwrite.cpp
    NALwrite.h
    SEIEncoder.cpp
    SEIEncoder.h
    SEIwrite.cpp
    SEIwrite.h
    SyntaxElementWriter.cpp
    SyntaxElementWriter.h
    TEncAnalyze.h
    TEncBinCoder.h
    TEncBinCoderCABAC.cpp
    TEncBinCoderCABAC.h
    TEncBinCoderCABACCounter.cpp
    TEncBinCoderCABACCounter.h
    TEncCavlc.cpp
    TEncCavlc.h
    TEncCfg.h
    TEncCu.cpp
    TEncCu.h
    TEncEntropy.cpp
    TEncEntropy.h
    TEncGOP.cpp
    TEncGOP.h
    TEncPic.cpp
    TEncPic.h
    TEncPreanalyzer.cpp
    TEncPreanalyzer.h
    TEncRateCtrl.cpp
    TEncRateCtrl.h
    TEncSampleAdaptiveOffset.cpp
    TEncSampleAdaptiveOffset.h
    TEncSbac.cpp
    TEncSbac.h
    TEncSearch.cpp
    TEncSearch.h
    TEncSlice.cpp
    TEncSlice.h
    TEncTop.cpp
    TEncTop.h
    WeightPredAnalysis.cpp
    WeightPredAnalysis.h)

ADD_LIBRARY(TLibEncoder STATIC ${SOURCE_FILES})
target_link_libraries(TLibEncoder TLibCommon)


