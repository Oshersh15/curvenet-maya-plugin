#include "CutPathMeshSplitter.h"

#include <gtest/gtest.h>
#include <algorithm>

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

TEST(
    CutPathMeshSplitter,
    AppliesStraightCutPathAcrossGrid
)
{
    HalfEdgeMesh mesh;
    mesh.createFourQuadGrid();

    CutPath cutPath;
    cutPath.curveId = 0;

    CutVertex leftCut;
    leftCut.position =
        Point3{0.0, 1.5, 0.0};
    leftCut.sourceHalfEdgeId = 3;
    leftCut.sourceEdgeT = 0.5;
    leftCut.curveId = 0;
    leftCut.cutPathOrder = 0;

    CutVertex centreCut;
    centreCut.position =
        Point3{1.0, 1.5, 0.0};
    centreCut.sourceHalfEdgeId = 1;
    centreCut.sourceEdgeT = 0.5;
    centreCut.curveId = 0;
    centreCut.cutPathOrder = 1;

    CutVertex rightCut;
    rightCut.position =
        Point3{2.0, 1.5, 0.0};
    rightCut.sourceHalfEdgeId = 5;
    rightCut.sourceEdgeT = 0.5;
    rightCut.curveId = 0;
    rightCut.cutPathOrder = 2;

    cutPath.cutVertices = {
        leftCut,
        centreCut,
        rightCut
    };

    const CutPathSplitResult result =
        CutPathMeshSplitter::apply(
            mesh,
            cutPath,
            0.0001
        );

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.meshVertexIds.size(),
        3
    );

    EXPECT_EQ(mesh.vertices.size(), 12);
    EXPECT_EQ(mesh.halfEdges.size(), 20);

    const int leftVertexId =
        result.meshVertexIds[0];

    const int centreVertexId =
        result.meshVertexIds[1];

    const int rightVertexId =
        result.meshVertexIds[2];

    EXPECT_DOUBLE_EQ(
        mesh.vertices[leftVertexId].position.x,
        0.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[leftVertexId].position.y,
        1.5
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[centreVertexId].position.x,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[centreVertexId].position.y,
        1.5
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[rightVertexId].position.x,
        2.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[rightVertexId].position.y,
        1.5
    );

    const std::vector<int> firstFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> secondFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    EXPECT_EQ(firstFaceHalfEdges.size(), 6);
    EXPECT_EQ(secondFaceHalfEdges.size(), 6);
}

TEST(
    CutPathMeshSplitter,
    AppliesDiagonalCutPathAcrossGrid
)
{
    HalfEdgeMesh mesh;
    mesh.createFourQuadGrid();

    CutPath cutPath;
    cutPath.curveId = 0;

    CutVertex leftCut;
    leftCut.position =
        Point3{0.0, 1.75, 0.0};
    leftCut.sourceHalfEdgeId = 3;
    leftCut.sourceEdgeT = 0.75;
    leftCut.curveId = 0;
    leftCut.cutPathOrder = 0;

    CutVertex verticalCut;
    verticalCut.position =
        Point3{1.0, 1.25, 0.0};
    verticalCut.sourceHalfEdgeId = 1;
    verticalCut.sourceEdgeT = 0.75;
    verticalCut.curveId = 0;
    verticalCut.cutPathOrder = 1;

    CutVertex horizontalCut;
    horizontalCut.position =
        Point3{1.5, 1.0, 0.0};
    horizontalCut.sourceHalfEdgeId = 6;
    horizontalCut.sourceEdgeT = 0.5;
    horizontalCut.curveId = 0;
    horizontalCut.cutPathOrder = 2;

    CutVertex rightCut;
    rightCut.position =
        Point3{2.0, 0.75, 0.0};
    rightCut.sourceHalfEdgeId = 13;
    rightCut.sourceEdgeT = 0.25;
    rightCut.curveId = 0;
    rightCut.cutPathOrder = 3;

    cutPath.cutVertices = {
        leftCut,
        verticalCut,
        horizontalCut,
        rightCut
    };

    const CutPathSplitResult result =
        CutPathMeshSplitter::apply(
            mesh,
            cutPath,
            0.0001
        );

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.meshVertexIds.size(),
        4
    );

    EXPECT_EQ(mesh.vertices.size(), 13);
    EXPECT_EQ(mesh.halfEdges.size(), 22);

    const int leftVertexId =
        result.meshVertexIds[0];

    const int verticalVertexId =
        result.meshVertexIds[1];

    const int horizontalVertexId =
        result.meshVertexIds[2];

    const int rightVertexId =
        result.meshVertexIds[3];

    EXPECT_DOUBLE_EQ(
        mesh.vertices[leftVertexId].position.x,
        0.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[leftVertexId].position.y,
        1.75
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[verticalVertexId].position.x,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[verticalVertexId].position.y,
        1.25
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[horizontalVertexId].position.x,
        1.5
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[horizontalVertexId].position.y,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[rightVertexId].position.x,
        2.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[rightVertexId].position.y,
        0.75
    );

    const std::vector<int> face0HalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> face1HalfEdges =
        mesh.getFaceHalfEdges(1);

    const std::vector<int> face3HalfEdges =
        mesh.getFaceHalfEdges(3);

    EXPECT_EQ(face0HalfEdges.size(), 6);
    EXPECT_EQ(face1HalfEdges.size(), 6);
    EXPECT_EQ(face3HalfEdges.size(), 6);

    /*
        Face 2 was not crossed by the path,
        so it should remain a quad.
    */
    const std::vector<int> face2HalfEdges =
        mesh.getFaceHalfEdges(2);

    EXPECT_EQ(face2HalfEdges.size(), 4);
}

TEST(
    CutPathMeshSplitter,
    CreatesTwoDirectedCutHalfEdges
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    /*
        Add two vertices representing CutVertices that have
        already been inserted into the mesh by the edge-splitting stage.
    */
    Vertex firstCutVertex;
    firstCutVertex.position =
        Point3{0.0, 0.5, 0.0};

    Vertex secondCutVertex;
    secondCutVertex.position =
        Point3{1.0, 0.5, 0.0};

    mesh.vertices.push_back(
        firstCutVertex
    );

    mesh.vertices.push_back(
        secondCutVertex
    );

    CutPath cutPath;

    /*
        There is one interval between the two CutVertices,
        and it belongs to face 0.
    */
    cutPath.faceIntervalIds.push_back(0);

    CutPathSplitResult splitResult;
    splitResult.success = true;

    splitResult.meshVertexIds = {
        4,
        5
    };

    const int originalHalfEdgeCount =
        static_cast<int>(
            mesh.halfEdges.size()
        );

    const CutHalfEdgePairResult result =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            splitResult.meshVertexIds[0],
            splitResult.meshVertexIds[1]
        );

    ASSERT_TRUE(result.success);

    EXPECT_EQ(
        mesh.halfEdges.size(),
        originalHalfEdgeCount + 2
    );

    const HalfEdge& firstHalfEdge =
        mesh.halfEdges[
            result.firstHalfEdgeId
        ];

    const HalfEdge& secondHalfEdge =
        mesh.halfEdges[
            result.secondHalfEdgeId
        ];

    EXPECT_EQ(
        firstHalfEdge.startVertex,
        4
    );

    EXPECT_EQ(
        firstHalfEdge.endVertex,
        5
    );

    EXPECT_EQ(
        secondHalfEdge.startVertex,
        5
    );

    EXPECT_EQ(
        secondHalfEdge.endVertex,
        4
    );

    EXPECT_EQ(
        firstHalfEdge.twin,
        result.secondHalfEdgeId
    );

    EXPECT_EQ(
        secondHalfEdge.twin,
        result.firstHalfEdgeId
    );

    EXPECT_EQ(firstHalfEdge.next, -1);
    EXPECT_EQ(secondHalfEdge.next, -1);

    EXPECT_EQ(firstHalfEdge.face, -1);
    EXPECT_EQ(secondHalfEdge.face, -1);
}

TEST(
    CutPathMeshSplitter,
    InsertsCutHalfEdgesAndSplitsFaceIntoTwoLoops
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    /*
        The two boundary splits created:

        vertex 4 = top CutVertex
        vertex 5 = bottom CutVertex
    */
    CutPath cutPath;

    cutPath.faceIntervalIds.push_back(
        0
    );

    CutPathSplitResult splitResult;
    splitResult.success = true;

    splitResult.meshVertexIds = {
        topSplit.newVertexId,
        bottomSplit.newVertexId
    };

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            splitResult.meshVertexIds[0],
            splitResult.meshVertexIds[1]
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_EQ(
        mesh.faces.size(),
        1
    );

    const bool inserted =
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        );

    ASSERT_TRUE(inserted);

    EXPECT_EQ(
        mesh.faces.size(),
        2
    );

    const std::vector<int> originalFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> newFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    EXPECT_EQ(
        originalFaceHalfEdges.size(),
        4
    );

    EXPECT_EQ(
        newFaceHalfEdges.size(),
        4
    );

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.firstHalfEdgeId
        ].face,
        1
    );

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.secondHalfEdgeId
        ].face,
        0
    );

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.firstHalfEdgeId
        ].twin,
        cutEdgeResult.secondHalfEdgeId
    );

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.secondHalfEdgeId
        ].twin,
        cutEdgeResult.firstHalfEdgeId
    );
}

TEST(
    HalfEdgeMesh,
    FindsCurrentFaceAfterFaceSplit
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    CutPath cutPath;
    cutPath.faceIntervalIds.push_back(0);

    CutPathSplitResult splitResult;
    splitResult.success = true;
    splitResult.meshVertexIds = {
        topSplit.newVertexId,
        bottomSplit.newVertexId
    };

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            splitResult.meshVertexIds[0],
            splitResult.meshVertexIds[1]
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    /*
        One side remains face 0.
        The other becomes face 1.

        Vertices 0 and the top cut vertex should now
        belong to one of those current faces.
    */
    const int foundFace =
        mesh.findFaceContainingVertices(
            0,
            topSplit.newVertexId
        );

    EXPECT_GE(foundFace, 0);
    EXPECT_LT(
        foundFace,
        static_cast<int>(mesh.faces.size())
    );
}

TEST(
    CutPathMeshSplitter,
    SplitsQuadAcrossOppositeEdgesIntoTwoQuads
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            topSplit.newVertexId,
            bottomSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    ASSERT_EQ(mesh.faces.size(), 2);

    const std::vector<int> firstFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> secondFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    ASSERT_EQ(firstFaceHalfEdges.size(), 4);
    ASSERT_EQ(secondFaceHalfEdges.size(), 4);

    for (int halfEdgeId : firstFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            0
        );
    }

    for (int halfEdgeId : secondFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            1
        );
    }

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.firstHalfEdgeId
        ].twin,
        cutEdgeResult.secondHalfEdgeId
    );

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.secondHalfEdgeId
        ].twin,
        cutEdgeResult.firstHalfEdgeId
    );
}

TEST(
    CutPathMeshSplitter,
    SplitsQuadAcrossAdjacentEdgesIntoTriangleAndPentagon
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult rightSplit =
        mesh.splitBoundaryHalfEdge(
            1,
            Point3{1.0, 0.5, 0.0}
        );

    ASSERT_TRUE(rightSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            topSplit.newVertexId,
            rightSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    ASSERT_EQ(mesh.faces.size(), 2);

    const std::vector<int> firstFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> secondFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    const int firstFaceSize =
        static_cast<int>(
            firstFaceHalfEdges.size()
        );

    const int secondFaceSize =
        static_cast<int>(
            secondFaceHalfEdges.size()
        );

    const bool validFaceSizes =
        (
            firstFaceSize == 3 &&
            secondFaceSize == 5
        ) ||
        (
            firstFaceSize == 5 &&
            secondFaceSize == 3
        );

    EXPECT_TRUE(validFaceSizes);

    for (int halfEdgeId : firstFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            0
        );
    }

    for (int halfEdgeId : secondFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            1
        );
    }

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.firstHalfEdgeId
        ].twin,
        cutEdgeResult.secondHalfEdgeId
    );

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.secondHalfEdgeId
        ].twin,
        cutEdgeResult.firstHalfEdgeId
    );
}

TEST(
    CutPathMeshSplitter,
    SplitsTriangleAcrossTwoEdgesIntoTriangleAndQuad
)
{
    HalfEdgeMesh mesh;
    mesh.createTestTriangle();

    const BoundaryHalfEdgeSplitResult firstSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.75, 0.5, 0.0}
        );

    ASSERT_TRUE(firstSplit.success);

    const BoundaryHalfEdgeSplitResult secondSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.25, 0.5, 0.0}
        );

    ASSERT_TRUE(secondSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            firstSplit.newVertexId,
            secondSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    ASSERT_EQ(mesh.faces.size(), 2);

    const std::vector<int> firstFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> secondFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    const int firstFaceSize =
        static_cast<int>(
            firstFaceHalfEdges.size()
        );

    const int secondFaceSize =
        static_cast<int>(
            secondFaceHalfEdges.size()
        );

    const bool validFaceSizes =
        (
            firstFaceSize == 3 &&
            secondFaceSize == 4
        ) ||
        (
            firstFaceSize == 4 &&
            secondFaceSize == 3
        );

    EXPECT_TRUE(validFaceSizes);

    for (int halfEdgeId : firstFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            0
        );
    }

    for (int halfEdgeId : secondFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            1
        );
    }

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.firstHalfEdgeId
        ].twin,
        cutEdgeResult.secondHalfEdgeId
    );

    EXPECT_EQ(
        mesh.halfEdges[
            cutEdgeResult.secondHalfEdgeId
        ].twin,
        cutEdgeResult.firstHalfEdgeId
    );
}

TEST(
    CutPathMeshSplitter,
    BuildsFirstFaceLoopWhenDividingCrossedFace
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            topSplit.newVertexId,
            bottomSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    const std::vector<int> faceHalfEdges =
        mesh.getFaceHalfEdges(0);

    ASSERT_EQ(faceHalfEdges.size(), 4);

    /*
        Every edge must connect continuously to the next edge:

        current.endVertex == next.startVertex
    */
    for (int index = 0;
         index < static_cast<int>(faceHalfEdges.size());
         ++index)
    {
        const int currentHalfEdgeId =
            faceHalfEdges[index];

        const int nextHalfEdgeId =
            faceHalfEdges[
                (index + 1) %
                faceHalfEdges.size()
            ];

        EXPECT_EQ(
            mesh.halfEdges[
                currentHalfEdgeId
            ].endVertex,
            mesh.halfEdges[
                nextHalfEdgeId
            ].startVertex
        );

        EXPECT_EQ(
            mesh.halfEdges[
                currentHalfEdgeId
            ].next,
            nextHalfEdgeId
        );
    }
}

TEST(
    CutPathMeshSplitter,
    BuildsSecondFaceLoopWhenDividingCrossedFace
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            topSplit.newVertexId,
            bottomSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    ASSERT_EQ(mesh.faces.size(), 2);

    const std::vector<int> faceHalfEdges =
        mesh.getFaceHalfEdges(1);

    ASSERT_EQ(faceHalfEdges.size(), 4);

    for (int index = 0;
         index < static_cast<int>(
             faceHalfEdges.size()
         );
         ++index)
    {
        const int currentHalfEdgeId =
            faceHalfEdges[index];

        const int nextHalfEdgeId =
            faceHalfEdges[
                (index + 1) %
                faceHalfEdges.size()
            ];

        EXPECT_EQ(
            mesh.halfEdges[
                currentHalfEdgeId
            ].endVertex,
            mesh.halfEdges[
                nextHalfEdgeId
            ].startVertex
        );

        EXPECT_EQ(
            mesh.halfEdges[
                currentHalfEdgeId
            ].next,
            nextHalfEdgeId
        );
    }
}

TEST(
    CutPathMeshSplitter,
    AssignsFaceOwnershipAfterDividingCrossedFace
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            topSplit.newVertexId,
            bottomSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    ASSERT_EQ(mesh.faces.size(), 2);

    const std::vector<int> firstFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> secondFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    for (int halfEdgeId : firstFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            0
        );
    }

    for (int halfEdgeId : secondFaceHalfEdges)
    {
        EXPECT_EQ(
            mesh.halfEdges[halfEdgeId].face,
            1
        );
    }
}

TEST(
    CutPathMeshSplitter,
    UpdatesOriginalFaceRecordAfterDivision
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            topSplit.newVertexId,
            bottomSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    ASSERT_EQ(mesh.faces.size(), 2);

    const int originalFaceHalfEdgeId =
        mesh.faces[0].halfEdge;

    ASSERT_GE(
        originalFaceHalfEdgeId,
        0
    );

    ASSERT_LT(
        originalFaceHalfEdgeId,
        static_cast<int>(
            mesh.halfEdges.size()
        )
    );

    EXPECT_EQ(
        mesh.halfEdges[
            originalFaceHalfEdgeId
        ].face,
        0
    );

    const std::vector<int> originalFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    EXPECT_NE(
        std::find(
            originalFaceHalfEdges.begin(),
            originalFaceHalfEdges.end(),
            originalFaceHalfEdgeId
        ),
        originalFaceHalfEdges.end()
    );
}

TEST(
    CutPathMeshSplitter,
    AddsNewFaceRecordAfterDivision
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult topSplit =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(topSplit.success);

    const BoundaryHalfEdgeSplitResult bottomSplit =
        mesh.splitBoundaryHalfEdge(
            2,
            Point3{0.5, 0.0, 0.0}
        );

    ASSERT_TRUE(bottomSplit.success);

    const CutHalfEdgePairResult cutEdgeResult =
        CutPathMeshSplitter::createCutHalfEdges(
            mesh,
            topSplit.newVertexId,
            bottomSplit.newVertexId
        );

    ASSERT_TRUE(cutEdgeResult.success);

    ASSERT_EQ(mesh.faces.size(), 1);

    ASSERT_TRUE(
        CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
            mesh,
            0,
            cutEdgeResult.firstHalfEdgeId,
            cutEdgeResult.secondHalfEdgeId
        )
    );

    ASSERT_EQ(mesh.faces.size(), 2);

    const int newFaceId = 1;

    const int newFaceHalfEdgeId =
        mesh.faces[newFaceId].halfEdge;

    ASSERT_GE(
        newFaceHalfEdgeId,
        0
    );

    ASSERT_LT(
        newFaceHalfEdgeId,
        static_cast<int>(
            mesh.halfEdges.size()
        )
    );

    EXPECT_EQ(
        mesh.halfEdges[
            newFaceHalfEdgeId
        ].face,
        newFaceId
    );

    const std::vector<int> newFaceHalfEdges =
        mesh.getFaceHalfEdges(
            newFaceId
        );

    EXPECT_NE(
        std::find(
            newFaceHalfEdges.begin(),
            newFaceHalfEdges.end(),
            newFaceHalfEdgeId
        ),
        newFaceHalfEdges.end()
    );
}
