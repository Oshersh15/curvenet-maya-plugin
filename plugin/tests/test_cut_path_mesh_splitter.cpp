#include "CutPathMeshSplitter.h"

#include <gtest/gtest.h>

TEST(
    CutPathMeshSplitter,
    ProcessesCutVerticesInTraversalOrder
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutPath cutPath;

    CutVertex laterCut;
    laterCut.position =
        Point3{0.5, 0.0, 0.0};
    laterCut.sourceHalfEdgeId = 2;
    laterCut.sourceEdgeT = 0.5;
    laterCut.curveId = 0;
    laterCut.cutPathOrder = 1;

    CutVertex earlierCut;
    earlierCut.position =
        Point3{0.5, 1.0, 0.0};
    earlierCut.sourceHalfEdgeId = 0;
    earlierCut.sourceEdgeT = 0.5;
    earlierCut.curveId = 0;
    earlierCut.cutPathOrder = 0;

    /*
        Store them deliberately in the wrong vector order.

        Vector index 0 has CutPath order 1.
        Vector index 1 has CutPath order 0.
    */
    cutPath.cutVertices.push_back(
        laterCut
    );

    cutPath.cutVertices.push_back(
        earlierCut
    );

    const CutPathSplitResult result =
        CutPathMeshSplitter::apply(
            mesh,
            cutPath,
            0.0001
        );

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.meshVertexIds.size(), 2);

    /*
        The order-0 cut must be processed first,
        so it creates mesh vertex 4.

        It came from original vector index 1.
    */
    EXPECT_EQ(
        result.meshVertexIds[1],
        4
    );

    /*
        The order-1 cut is processed second,
        so it creates mesh vertex 5.

        It came from original vector index 0.
    */
    EXPECT_EQ(
        result.meshVertexIds[0],
        5
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[4].position.x,
        0.5
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[4].position.y,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[5].position.x,
        0.5
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[5].position.y,
        0.0
    );
}

TEST(
    CutPathMeshSplitter,
    RedirectsLaterCutsToRemainingEdgeSegment
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutPath cutPath;

    CutVertex firstCut;
    firstCut.position =
        Point3{0.25, 1.0, 0.0};
    firstCut.sourceHalfEdgeId = 0;
    firstCut.sourceEdgeT = 0.25;
    firstCut.curveId = 0;
    firstCut.cutPathOrder = 0;

    CutVertex secondCut;
    secondCut.position =
        Point3{0.75, 1.0, 0.0};
    secondCut.sourceHalfEdgeId = 0;
    secondCut.sourceEdgeT = 0.75;
    secondCut.curveId = 0;
    secondCut.cutPathOrder = 1;

    cutPath.cutVertices.push_back(
        firstCut
    );

    cutPath.cutVertices.push_back(
        secondCut
    );

    const CutPathSplitResult result =
        CutPathMeshSplitter::apply(
            mesh,
            cutPath,
            0.0001
        );

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.meshVertexIds.size(), 2);

    EXPECT_EQ(
        result.meshVertexIds[0],
        4
    );

    EXPECT_EQ(
        result.meshVertexIds[1],
        5
    );

    /*
        The original half-edge was shortened only once:

        vertex 0 -> vertex 4
    */
    EXPECT_EQ(
        mesh.halfEdges[0].startVertex,
        0
    );

    EXPECT_EQ(
        mesh.halfEdges[0].endVertex,
        4
    );

    /*
        The edge created by the first split was then
        redirected and split by the second CutVertex:

        vertex 4 -> vertex 5
    */
    EXPECT_EQ(
        mesh.halfEdges[4].startVertex,
        4
    );

    EXPECT_EQ(
        mesh.halfEdges[4].endVertex,
        5
    );

    /*
        The final remaining segment is:

        vertex 5 -> vertex 1
    */
    EXPECT_EQ(
        mesh.halfEdges[5].startVertex,
        5
    );

    EXPECT_EQ(
        mesh.halfEdges[5].endVertex,
        1
    );

    const std::vector<int> faceHalfEdges =
        mesh.getFaceHalfEdges(0);

    EXPECT_EQ(
        faceHalfEdges.size(),
        6
    );
}

TEST(
    CutPathMeshSplitter,
    ReusesMeshVertexForDuplicateCutPosition
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutPath cutPath;

    CutVertex firstCut;
    firstCut.position =
        Point3{0.5, 1.0, 0.0};
    firstCut.sourceHalfEdgeId = 0;
    firstCut.sourceEdgeT = 0.5;
    firstCut.curveId = 0;
    firstCut.cutPathOrder = 0;

    CutVertex duplicateCut;
    duplicateCut.position =
        Point3{0.5, 1.0, 0.0};
    duplicateCut.sourceHalfEdgeId = 0;
    duplicateCut.sourceEdgeT = 0.5;
    duplicateCut.curveId = 0;
    duplicateCut.cutPathOrder = 1;

    cutPath.cutVertices.push_back(
        firstCut
    );

    cutPath.cutVertices.push_back(
        duplicateCut
    );

    const CutPathSplitResult result =
        CutPathMeshSplitter::apply(
            mesh,
            cutPath,
            0.0001
        );

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.meshVertexIds.size(),
        2
    );

    EXPECT_EQ(
        result.meshVertexIds[0],
        4
    );

    EXPECT_EQ(
        result.meshVertexIds[1],
        4
    );

    EXPECT_EQ(
        mesh.vertices.size(),
        5
    );

    EXPECT_EQ(
        mesh.halfEdges.size(),
        5
    );
}
