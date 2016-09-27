cmake_minimum_required(VERSION 2.8)
project(BitrateTargeting)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../../../Lib/
)

set(SOURCE_FILES_EXTRACT
        ExtractBitratesMain.cpp
        ExtractBitrates.cpp
        )

add_executable(ExtractBitrates ${SOURCE_FILES_EXTRACT})
target_link_libraries(ExtractBitrates TAppCommon TLibCommon TLibEncoder TLibDecoder TLibVideoIO)

set(SOURCE_FILES_GUESS
        GuessLambdaModifiersMain.cpp
        GuessLambdaModifiers.cpp
        )

add_executable(GuessLambdaModifiers ${SOURCE_FILES_GUESS})
target_link_libraries(GuessLambdaModifiers TAppCommon TLibCommon TLibEncoder TLibDecoder TLibVideoIO)
