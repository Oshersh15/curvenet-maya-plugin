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
