cmake_minimum_required(VERSION 3.16)

project(CurvenetPlugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MAYA_LOCATION "/Applications/Autodesk/maya2025")

add_library(CurvenetPlugin SHARED
    src/CurvenetNode.cpp
    src/CurvenetData.cpp
)

target_include_directories(CurvenetPlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(CurvenetPlugin PRIVATE
    "${MAYA_LOCATION}/include"
)

target_link_directories(CurvenetPlugin PRIVATE
    "${MAYA_LOCATION}/Maya.app/Contents/MacOS"
)

target_link_libraries(CurvenetPlugin
    OpenMaya
    OpenMayaAnim
    Foundation
)

set_target_properties(CurvenetPlugin PROPERTIES
    PREFIX ""
    SUFFIX ".bundle"
)
