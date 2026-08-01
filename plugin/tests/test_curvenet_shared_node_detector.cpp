#include <gtest/gtest.h>

#include "CurvenetSharedNodeDetector.h"

TEST(
    CurvenetSharedNodeDetector,
    FindsMatchingEndpointOnExistingOpenChain
)
{
    CurvenetCutResult curvenetResult;

    curvenetResult.mesh.vertices.resize(2);

    curvenetResult.mesh.vertices[0].position =
        Point3{0.0, 0.0, 0.0};

    curvenetResult.mesh.vertices[1].position =
        Point3{1.0, 0.0, 0.0};

    CutChain existingChain;
    existingChain.curveId = 0;
    existingChain.closed = false;
    existingChain.vertexIds = {
        0,
        1
    };

    curvenetResult.cutChainsByCurveId[0] =
        existingChain;

    CutVertex incomingEndpoint;
    incomingEndpoint.position =
        Point3{1.00001, 0.0, 0.0};

    const std::optional<int> sharedVertexId =
        CurvenetSharedNodeDetector::
            findSharedMeshVertex(
                incomingEndpoint,
                curvenetResult,
                0.001
            );

    ASSERT_TRUE(
        sharedVertexId.has_value()
    );

    EXPECT_EQ(
        sharedVertexId.value(),
        1
    );
}

TEST(
    CurvenetSharedNodeDetector,
    ReturnsNoMatchWhenEndpointIsOutsideTolerance
)
{
    CurvenetCutResult curvenetResult;

    curvenetResult.mesh.vertices.resize(2);

    curvenetResult.mesh.vertices[0].position =
        Point3{0.0, 0.0, 0.0};

    curvenetResult.mesh.vertices[1].position =
        Point3{1.0, 0.0, 0.0};

    CutChain existingChain;
    existingChain.curveId = 0;
    existingChain.closed = false;
    existingChain.vertexIds = {
        0,
        1
    };

    curvenetResult.cutChainsByCurveId[0] =
        existingChain;

    CutVertex incomingEndpoint;
    incomingEndpoint.position =
        Point3{1.1, 0.0, 0.0};

    const std::optional<int> sharedVertexId =
        CurvenetSharedNodeDetector::
            findSharedMeshVertex(
                incomingEndpoint,
                curvenetResult,
                0.001
            );

    EXPECT_FALSE(
        sharedVertexId.has_value()
    );
}

TEST(
    CurvenetSharedNodeDetector,
    FindsMatchingInteriorVertexOnExistingChain
)
{
    CurvenetCutResult curvenetResult;

    curvenetResult.mesh.vertices.resize(3);

    curvenetResult.mesh.vertices[0].position =
        Point3{0.0, 0.0, 0.0};

    curvenetResult.mesh.vertices[1].position =
        Point3{1.0, 0.0, 0.0};

    curvenetResult.mesh.vertices[2].position =
        Point3{2.0, 0.0, 0.0};

    CutChain existingChain;
    existingChain.curveId = 0;
    existingChain.closed = false;

    existingChain.vertexIds = {
        0,
        1,
        2
    };

    curvenetResult.cutChainsByCurveId[0] =
        existingChain;

    CutVertex incomingEndpoint;
    incomingEndpoint.position =
        Point3{1.00001, 0.0, 0.0};

    const std::optional<int> sharedVertexId =
        CurvenetSharedNodeDetector::
            findSharedMeshVertex(
                incomingEndpoint,
                curvenetResult,
                0.001
            );

    ASSERT_TRUE(
        sharedVertexId.has_value()
    );

    /*
        The incoming endpoint should match the
        interior vertex of the existing CutChain.
    */
    EXPECT_EQ(
        sharedVertexId.value(),
        1
    );
}

TEST(
    CurvenetSharedNodeDetector,
    FindsMatchingVertexOnExistingClosedChain
)
{
    CurvenetCutResult curvenetResult;

    curvenetResult.mesh.vertices.resize(3);

    curvenetResult.mesh.vertices[0].position =
        Point3{0.0, 0.0, 0.0};

    curvenetResult.mesh.vertices[1].position =
        Point3{1.0, 0.0, 0.0};

    curvenetResult.mesh.vertices[2].position =
        Point3{0.5, 1.0, 0.0};

    CutChain existingChain;
    existingChain.curveId = 0;
    existingChain.closed = true;

    existingChain.vertexIds = {
        0,
        1,
        2
    };

    curvenetResult.cutChainsByCurveId[0] =
        existingChain;

    CutVertex incomingEndpoint;
    incomingEndpoint.position =
        Point3{0.5, 1.00001, 0.0};

    const std::optional<int> sharedVertexId =
        CurvenetSharedNodeDetector::
            findSharedMeshVertex(
                incomingEndpoint,
                curvenetResult,
                0.001
            );

    ASSERT_TRUE(
        sharedVertexId.has_value()
    );

    /*
        Closed CutChains can also contain
        shared Curvenet nodes.
    */
    EXPECT_EQ(
        sharedVertexId.value(),
        2
    );
}
