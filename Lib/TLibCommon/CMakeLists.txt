cmake_minimum_required(VERSION 2.8)
project(TLibCommon)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../
)

set(SOURCE_FILES
    AccessUnit.h
    CommonDef.h
    ContextModel.cpp
    ContextModel.h
    ContextModel3DBuffer.cpp
    ContextModel3DBuffer.h
    ContextTables.h
    Debug.cpp
    Debug.h
    NAL.h
    SEI.cpp
    SEI.h
    TComBitCounter.h
    TComBitStream.cpp
    TComBitStream.h
    TComCABACTables.cpp
    TComCABACTables.h
    TComChromaFormat.cpp
    TComChromaFormat.h
    TComCodingStatistics.h
    TComDataCU.cpp
    TComDataCU.h
    TComHash.cpp
    TComHash.h
    TComInterpolationFilter.cpp
    TComInterpolationFilter.h
    TComList.h
    TComLoopFilter.cpp
    TComLoopFilter.h
    TComMotionInfo.cpp
    TComMotionInfo.h
    TComMv.h
    TComPattern.cpp
    TComPattern.h
    TComPic.cpp
    TComPic.h
    TComPicSym.cpp
    TComPicSym.h
    TComPicYuv.cpp
    TComPicYuv.h
    TComPicYuvMD5.cpp
    TComPrediction.cpp
    TComPrediction.h
    TComRdCost.cpp
    TComRdCost.h
    TComRdCostWeightPrediction.cpp
    TComRdCostWeightPrediction.h
    TComRectangle.h
    TComRom.cpp
    TComRom.h
    TComSampleAdaptiveOffset.cpp
    TComSampleAdaptiveOffset.h
    TComSlice.cpp
    TComSlice.h
    TComTrQuant.cpp
    TComTrQuant.h
    TComTU.cpp
    TComTU.h
    TComWeightPrediction.cpp
    TComWeightPrediction.h
    TComYuv.cpp
    TComYuv.h
    TypeDef.h)

ADD_LIBRARY(TLibCommon STATIC ${SOURCE_FILES})
target_link_libraries(TLibCommon libmd5)