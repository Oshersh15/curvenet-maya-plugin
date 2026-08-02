cmake_minimum_required(VERSION 3.16)

project(CurvenetPlugin)

set(
    MAYA_LOCATION
    "/Applications/Autodesk/maya2025"
    CACHE PATH
    "Path to the Maya installation"
)

if(APPLE)
    set(
        MAYA_INCLUDE_DIR
        "${MAYA_LOCATION}/include"
    )

    set(
        MAYA_LIBRARY_DIR
        "${MAYA_LOCATION}/Maya.app/Contents/MacOS"
    )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(CurvenetPlugin SHARED
    src/CurvenetNode.cpp
    src/CurvenetData.cpp
    src/MayaMeshConverter.cpp
    src/ProfileCurveSampler.cpp
    src/GeometryUtils.cpp
    src/CurveMeshIntersector.cpp
    src/CurvenetDebugCommand.cpp
    src/HalfEdge.cpp
    src/CurveConnectionDetector.cpp
    src/CutPathMeshSplitter.cpp
    src/CurvenetMeshCutter.cpp
    src/CurvenetSharedNodeDetector.cpp
    src/CurvenetFaceBuilder.cpp
    src/CurvenetFaceRegionBuilder.cpp
)

target_include_directories(CurvenetPlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MAYA_INCLUDE_DIR}
)

target_link_directories(CurvenetPlugin PRIVATE
    ${MAYA_LIBRARY_DIR}
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
    tests/test_geometry_utils.cpp
    src/GeometryUtils.cpp
    src/CurveMeshIntersector.cpp
    tests/test_curve_mesh_intersector.cpp
    src/HalfEdge.cpp
    tests/test_curve_connection_detector.cpp
    src/CurveConnectionDetector.cpp
    tests/test_cut_vertex.cpp
    tests/test_cut_path_mesh_splitter.cpp
    src/CutPathMeshSplitter.cpp
    src/CurvenetMeshCutter.cpp
    tests/test_curvenet_mesh_cutter.cpp
    src/CurvenetSharedNodeDetector.cpp
    tests/test_curvenet_shared_node_detector.cpp
    src/CurvenetFaceBuilder.cpp
    tests/test_curvenet_face_builder.cpp
    src/CurvenetFaceRegionBuilder.cpp
    tests/test_curvenet_face_region_builder.cpp
)

target_include_directories(HalfEdgeTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(HalfEdgeTests
    GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(HalfEdgeTests)
