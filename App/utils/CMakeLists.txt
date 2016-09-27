cmake_minimum_required(VERSION 2.8)
project(utils)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

INCLUDE_DIRECTORIES(
        ../../Lib/
)

set(SOURCE_FILES_1
        annexBbytecount.cpp
    )

add_executable(annexBbytecount ${SOURCE_FILES_1})
target_link_libraries(annexBbytecount TAppCommon TLibCommon TLibEncoder TLibDecoder TLibVideoIO)

set(SOURCE_FILES_2
        convert_NtoMbit_YCbCr.cpp
        )

add_executable(convert_NtoMbit_YCbCr ${SOURCE_FILES_2})
target_link_libraries(convert_NtoMbit_YCbCr TAppCommon TLibCommon TLibEncoder TLibDecoder TLibVideoIO)
