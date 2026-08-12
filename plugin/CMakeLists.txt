cmake_minimum_required(VERSION 3.16)

project(CurvenetPlugin)

if(APPLE)
    set(DEFAULT_MAYA_LOCATION "/Applications/Autodesk/maya2025")
elseif(UNIX)
    set(DEFAULT_MAYA_LOCATION "/usr/autodesk/maya2025")
else()
    set(DEFAULT_MAYA_LOCATION "C:/Program Files/Autodesk/Maya2025")
endif()

set(
    MAYA_LOCATION
    "${DEFAULT_MAYA_LOCATION}"
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
    set(MAYA_PLUGIN_SUFFIX ".bundle")
elseif(UNIX)
    set(MAYA_INCLUDE_DIR "${MAYA_LOCATION}/include")
    set(MAYA_LIBRARY_DIR "${MAYA_LOCATION}/lib")
    set(MAYA_PLUGIN_SUFFIX ".so")
else()
    set(MAYA_INCLUDE_DIR "${MAYA_LOCATION}/include")
    set(MAYA_LIBRARY_DIR "${MAYA_LOCATION}/lib")
    set(MAYA_PLUGIN_SUFFIX ".mll")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(UNIX AND NOT APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 11.0 OR
       CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0)
        message(FATAL_ERROR
            "Maya 2025 Linux plugins must be built with Autodesk's supported "
            "DTS-11/GCC 11 toolchain. Detected GCC "
            "${CMAKE_CXX_COMPILER_VERSION}."
        )
    endif()
endif()

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
    src/CurvenetEdgeBuilder.cpp
    src/CurvenetSceneBuilder.cpp
    src/CurvenetRegionPreviewBuilder.cpp
    src/CurvenetHarmonicSolver.cpp
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
    SUFFIX "${MAYA_PLUGIN_SUFFIX}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

foreach(CONFIGURATION DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(CurvenetPlugin PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION} "${CMAKE_BINARY_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION} "${CMAKE_BINARY_DIR}"
    )
endforeach()

if(APPLE)
    target_compile_definitions(CurvenetPlugin PRIVATE OSMac_ OSMac_MachO_)
elseif(UNIX)
    target_compile_definitions(
        CurvenetPlugin
        PRIVATE
        _LINUX
        LINUX
        REQUIRE_IOSTREAM
    )
else()
    target_compile_definitions(
        CurvenetPlugin
        PRIVATE
        NT_PLUGIN
        REQUIRE_IOSTREAM
    )
endif()

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
    tests/test_curvenet_authored_grid.cpp
    src/CurvenetSharedNodeDetector.cpp
    tests/test_curvenet_shared_node_detector.cpp
    src/CurvenetFaceBuilder.cpp
    tests/test_curvenet_face_builder.cpp
    src/CurvenetFaceRegionBuilder.cpp
    tests/test_curvenet_face_region_builder.cpp
    src/CurvenetHarmonicSolver.cpp
    tests/test_curvenet_harmonic_solver.cpp
)

target_include_directories(HalfEdgeTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(HalfEdgeTests
    GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(HalfEdgeTests)
