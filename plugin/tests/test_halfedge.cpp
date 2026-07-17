#include "HalfEdge.h"

#include <gtest/gtest.h>

TEST(HalfEdgeMesh, CreateTestQuad)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    EXPECT_EQ(mesh.vertices.size(), 4);
    EXPECT_EQ(mesh.faces.size(), 1);
    EXPECT_EQ(mesh.halfEdges.size(), 4);
}

TEST(HalfEdgeMesh, TraverseTestQuadFace)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<int> traversal = mesh.traverseFace(0);

    ASSERT_EQ(traversal.size(), 4);
    EXPECT_EQ(traversal[0], 0);
    EXPECT_EQ(traversal[1], 1);
    EXPECT_EQ(traversal[2], 2);
    EXPECT_EQ(traversal[3], 3);
}

TEST(HalfEdgeMesh, AssignTwinsInTwoQuadMesh)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    EXPECT_EQ(mesh.halfEdges[1].twin, 7);
    EXPECT_EQ(mesh.halfEdges[7].twin, 1);
}

TEST(HalfEdgeMesh, TraverseFaceReturnsEmptyForInvalidFace)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<int> negativeTraversal = mesh.traverseFace(-1);
    std::vector<int> outOfRangeTraversal = mesh.traverseFace(99);

    EXPECT_TRUE(negativeTraversal.empty());
    EXPECT_TRUE(outOfRangeTraversal.empty());
}

TEST(HalfEdgeMesh, GetFaceHalfEdgesReturnsEmptyForInvalidFace)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<int> negativeFaceEdges = mesh.getFaceHalfEdges(-1);
    std::vector<int> outOfRangeFaceEdges = mesh.getFaceHalfEdges(99);

    EXPECT_TRUE(negativeFaceEdges.empty());
    EXPECT_TRUE(outOfRangeFaceEdges.empty());
}

TEST(HalfEdgeMesh, SingleQuadHasNoTwinEdges)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();
    mesh.assignTwins();

    for (const HalfEdge& halfEdge : mesh.halfEdges)
    {
        EXPECT_EQ(halfEdge.twin, -1);
    }
}

TEST(HalfEdgeMesh, TwoQuadMeshOnlySharedEdgeHasTwins)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    EXPECT_EQ(mesh.halfEdges[1].twin, 7);
    EXPECT_EQ(mesh.halfEdges[7].twin, 1);

    EXPECT_EQ(mesh.halfEdges[0].twin, -1);
    EXPECT_EQ(mesh.halfEdges[2].twin, -1);
    EXPECT_EQ(mesh.halfEdges[3].twin, -1);
    EXPECT_EQ(mesh.halfEdges[4].twin, -1);
    EXPECT_EQ(mesh.halfEdges[5].twin, -1);
    EXPECT_EQ(mesh.halfEdges[6].twin, -1);
}

TEST(HalfEdgeMesh, GetAdjacentFacesInTwoQuadMesh)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    std::vector<int> face0Adjacent = mesh.getAdjacentFaces(0);
    std::vector<int> face1Adjacent = mesh.getAdjacentFaces(1);

    ASSERT_EQ(face0Adjacent.size(), 1);
    ASSERT_EQ(face1Adjacent.size(), 1);

    EXPECT_EQ(face0Adjacent[0], 1);
    EXPECT_EQ(face1Adjacent[0], 0);
}

TEST(HalfEdgeMesh, SingleQuadMeanEdgeLengthIsOne)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    double meanEdgeLength = mesh.computeMeanEdgeLength();

    EXPECT_DOUBLE_EQ(meanEdgeLength, 1.0);
}

TEST(HalfEdgeMesh, EmptyMeshMeanEdgeLengthIsZero)
{
    HalfEdgeMesh mesh;

    double meanEdgeLength = mesh.computeMeanEdgeLength();

    EXPECT_DOUBLE_EQ(meanEdgeLength, 0.0);
}

TEST(HalfEdgeMesh, CollectUniqueVerticesFromFaces)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    std::vector<int> faceIds{
        0,
        1
    };

    std::vector<int> vertexIds =
        mesh.collectUniqueVerticesFromFaces(
            faceIds
        );

    ASSERT_EQ(vertexIds.size(), 6);

    EXPECT_EQ(vertexIds[0], 0);
    EXPECT_EQ(vertexIds[1], 1);
    EXPECT_EQ(vertexIds[2], 6);
    EXPECT_EQ(vertexIds[3], 5);
    EXPECT_EQ(vertexIds[4], 2);
    EXPECT_EQ(vertexIds[5], 7);
}

TEST(HalfEdgeMesh, SplitBoundaryHalfEdge)
{
    HalfEdgeMesh mesh;

    mesh.vertices.resize(2);

    mesh.vertices[0].position =
        Point3{0.0, 0.0, 0.0};

    mesh.vertices[1].position =
        Point3{2.0, 0.0, 0.0};

    HalfEdge boundaryEdge;

    boundaryEdge.startVertex = 0;
    boundaryEdge.endVertex = 1;
    boundaryEdge.next = -1;
    boundaryEdge.twin = -1;
    boundaryEdge.face = 0;

    mesh.halfEdges.push_back(
        boundaryEdge
    );

    Face face;
    face.halfEdge = 0;

    mesh.faces.push_back(face);

    BoundaryHalfEdgeSplitResult result =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{1.0, 0.0, 0.0}
        );

    ASSERT_TRUE(result.success);

    EXPECT_EQ(
        mesh.vertices.size(),
        3
    );

    EXPECT_EQ(
        mesh.halfEdges.size(),
        2
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[2].position.x,
        1.0
    );

    EXPECT_EQ(
        mesh.halfEdges[0].startVertex,
        0
    );

    EXPECT_EQ(
        mesh.halfEdges[0].endVertex,
        2
    );

    EXPECT_EQ(
        mesh.halfEdges[0].next,
        1
    );

    EXPECT_EQ(
        mesh.halfEdges[1].startVertex,
        2
    );

    EXPECT_EQ(
        mesh.halfEdges[1].endVertex,
        1
    );

    EXPECT_EQ(
        mesh.halfEdges[1].face,
        0
    );

    EXPECT_EQ(
        mesh.vertices[2].outgoingHalfEdge,
        1
    );
}

TEST(HalfEdgeMesh, SplitBoundaryHalfEdgeRejectsInvalidIndex)
{
    HalfEdgeMesh mesh;

    const BoundaryHalfEdgeSplitResult result =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{1.0, 0.0, 0.0}
        );

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.newVertexId, -1);
    EXPECT_EQ(result.firstHalfEdgeId, -1);
    EXPECT_EQ(result.secondHalfEdgeId, -1);
}

TEST(HalfEdgeMesh, SplitBoundaryHalfEdgeRejectsNonBoundaryEdge)
{
    HalfEdgeMesh mesh;

    mesh.vertices.resize(2);

    HalfEdge firstHalfEdge;
    firstHalfEdge.startVertex = 0;
    firstHalfEdge.endVertex = 1;
    firstHalfEdge.twin = 1;
    firstHalfEdge.face = 0;

    HalfEdge secondHalfEdge;
    secondHalfEdge.startVertex = 1;
    secondHalfEdge.endVertex = 0;
    secondHalfEdge.twin = 0;
    secondHalfEdge.face = 1;

    mesh.halfEdges.push_back(firstHalfEdge);
    mesh.halfEdges.push_back(secondHalfEdge);

    const BoundaryHalfEdgeSplitResult result =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 0.0, 0.0}
        );

    EXPECT_FALSE(result.success);

    EXPECT_EQ(
        mesh.vertices.size(),
        2
    );

    EXPECT_EQ(
        mesh.halfEdges.size(),
        2
    );
}
