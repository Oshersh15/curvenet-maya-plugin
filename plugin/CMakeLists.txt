cmake_minimum_required(VERSION 3.16)

project(CurvenetPlugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MAYA_LOCATION "/Applications/Autodesk/maya2025")

add_library(CurvenetPlugin SHARED
    src/CurvenetNode.cpp
    src/CurvenetData.cpp
    src/MayaMeshConverter.cpp
    src/ProfileCurveSampler.cpp
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

enable_testing()

find_package(GTest REQUIRED)

add_executable(HalfEdgeTests
    tests/test_halfedge.cpp
    tests/test_profile_curve_sampler.cpp
    src/ProfileCurveSampler.cpp
)

target_include_directories(HalfEdgeTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_directories(HalfEdgeTests PRIVATE
    "${MAYA_LOCATION}/Maya.app/Contents/MacOS"
)

target_link_libraries(HalfEdgeTests
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(HalfEdgeTests)
