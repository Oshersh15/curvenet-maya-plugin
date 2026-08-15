/* Tests half-edge construction, traversal, validation, and splitting. */

#include "HalfEdge.h"

#include <gtest/gtest.h>
#include <algorithm>

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

TEST(HalfEdgeMesh, SplitsBoundaryEdgeOfSingleQuad)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const BoundaryHalfEdgeSplitResult result =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(result.success);

    EXPECT_EQ(mesh.vertices.size(), 5);
    EXPECT_EQ(mesh.halfEdges.size(), 5);

    const std::vector<int> faceHalfEdges =
        mesh.getFaceHalfEdges(0);

    ASSERT_EQ(faceHalfEdges.size(), 5);

    EXPECT_EQ(mesh.halfEdges[0].startVertex, 0);
    EXPECT_EQ(mesh.halfEdges[0].endVertex, 4);
    EXPECT_EQ(mesh.halfEdges[0].next, 4);

    EXPECT_EQ(mesh.halfEdges[4].startVertex, 4);
    EXPECT_EQ(mesh.halfEdges[4].endVertex, 1);
    EXPECT_EQ(mesh.halfEdges[4].next, 1);
    EXPECT_EQ(mesh.halfEdges[4].face, 0);
    EXPECT_EQ(mesh.halfEdges[4].twin, -1);

    EXPECT_EQ(mesh.vertices[4].outgoingHalfEdge, 4);
    EXPECT_EQ(mesh.faces[0].halfEdge, 0);
}

TEST(HalfEdgeMesh, SplitsBoundaryEdgeOfTwoQuadGrid)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    const int originalSecondFaceHalfEdge =
        mesh.faces[1].halfEdge;

    const BoundaryHalfEdgeSplitResult result =
        mesh.splitBoundaryHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    ASSERT_TRUE(result.success);

    EXPECT_EQ(mesh.vertices.size(), 9);
    EXPECT_EQ(mesh.halfEdges.size(), 9);

    const std::vector<int> firstFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> secondFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    EXPECT_EQ(firstFaceHalfEdges.size(), 5);
    EXPECT_EQ(secondFaceHalfEdges.size(), 4);

    EXPECT_EQ(mesh.halfEdges[0].startVertex, 0);
    EXPECT_EQ(mesh.halfEdges[0].endVertex, 8);
    EXPECT_EQ(mesh.halfEdges[0].next, 8);

    EXPECT_EQ(mesh.halfEdges[8].startVertex, 8);
    EXPECT_EQ(mesh.halfEdges[8].endVertex, 1);
    EXPECT_EQ(mesh.halfEdges[8].next, 1);
    EXPECT_EQ(mesh.halfEdges[8].face, 0);
    EXPECT_EQ(mesh.halfEdges[8].twin, -1);

    EXPECT_EQ(mesh.faces[1].halfEdge, originalSecondFaceHalfEdge);

    EXPECT_EQ(mesh.halfEdges[1].twin, 7);
    EXPECT_EQ(mesh.halfEdges[7].twin, 1);
}

TEST(HalfEdgeMesh, SplitsInternalEdgePair)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    /*
        Shared internal edge:

        half-edge 1: vertex 1 -> vertex 6
        half-edge 7: vertex 6 -> vertex 1
    */
    const InternalHalfEdgeSplitResult result =
        mesh.splitInternalHalfEdge(
            1,
            Point3{1.0, 0.5, 0.0}
        );

    ASSERT_TRUE(result.success);

    EXPECT_EQ(mesh.vertices.size(), 9);
    EXPECT_EQ(mesh.halfEdges.size(), 10);

    EXPECT_EQ(result.newVertexId, 8);
    EXPECT_EQ(result.firstHalfEdgeId, 1);
    EXPECT_EQ(result.firstNewHalfEdgeId, 8);
    EXPECT_EQ(result.twinHalfEdgeId, 7);
    EXPECT_EQ(result.twinNewHalfEdgeId, 9);

    EXPECT_DOUBLE_EQ(
        mesh.vertices[8].position.x,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[8].position.y,
        0.5
    );

    EXPECT_DOUBLE_EQ(
        mesh.vertices[8].position.z,
        0.0
    );

    /*
        First face:

        original half-edge 1:
            1 -> 6

        becomes:
            1 -> 8
            8 -> 6
    */
    EXPECT_EQ(mesh.halfEdges[1].startVertex, 1);
    EXPECT_EQ(mesh.halfEdges[1].endVertex, 8);
    EXPECT_EQ(mesh.halfEdges[1].next, 8);
    EXPECT_EQ(mesh.halfEdges[1].face, 0);

    EXPECT_EQ(mesh.halfEdges[8].startVertex, 8);
    EXPECT_EQ(mesh.halfEdges[8].endVertex, 6);
    EXPECT_EQ(mesh.halfEdges[8].next, 2);
    EXPECT_EQ(mesh.halfEdges[8].face, 0);

    /*
        Second face:

        original twin half-edge 7:
            6 -> 1

        becomes:
            6 -> 8
            8 -> 1
    */
    EXPECT_EQ(mesh.halfEdges[7].startVertex, 6);
    EXPECT_EQ(mesh.halfEdges[7].endVertex, 8);
    EXPECT_EQ(mesh.halfEdges[7].next, 9);
    EXPECT_EQ(mesh.halfEdges[7].face, 1);

    EXPECT_EQ(mesh.halfEdges[9].startVertex, 8);
    EXPECT_EQ(mesh.halfEdges[9].endVertex, 1);
    EXPECT_EQ(mesh.halfEdges[9].next, 4);
    EXPECT_EQ(mesh.halfEdges[9].face, 1);

    /*
        Twin pairs after splitting:

            1 -> 8  <->  8 -> 1
            8 -> 6  <->  6 -> 8
    */
    EXPECT_EQ(mesh.halfEdges[1].twin, 9);
    EXPECT_EQ(mesh.halfEdges[9].twin, 1);

    EXPECT_EQ(mesh.halfEdges[8].twin, 7);
    EXPECT_EQ(mesh.halfEdges[7].twin, 8);

    EXPECT_EQ(
        mesh.vertices[8].outgoingHalfEdge,
        8
    );
}

TEST(HalfEdgeMesh, SplitInternalHalfEdgeRejectsBoundaryEdge)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    const InternalHalfEdgeSplitResult result =
        mesh.splitInternalHalfEdge(
            0,
            Point3{0.5, 1.0, 0.0}
        );

    EXPECT_FALSE(result.success);

    EXPECT_EQ(mesh.vertices.size(), 4);
    EXPECT_EQ(mesh.halfEdges.size(), 4);
}

TEST(HalfEdgeMesh, SplitInternalHalfEdgeRejectsInvalidTwinRelationship)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    /*
        Corrupt the relationship deliberately:
        half-edge 1 still points to 7,
        but half-edge 7 no longer points back to 1.
    */
    mesh.halfEdges[7].twin = -1;

    const InternalHalfEdgeSplitResult result =
        mesh.splitInternalHalfEdge(
            1,
            Point3{1.0, 0.5, 0.0}
        );

    EXPECT_FALSE(result.success);

    EXPECT_EQ(mesh.vertices.size(), 8);
    EXPECT_EQ(mesh.halfEdges.size(), 8);
}

TEST(
    HalfEdgeMesh,
    InternalEdgeSplitPreservesBothFaceLoops
)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    const InternalHalfEdgeSplitResult result =
        mesh.splitInternalHalfEdge(
            1,
            Point3{1.0, 0.5, 0.0}
        );

    ASSERT_TRUE(result.success);

    const std::vector<int> firstFaceHalfEdges =
        mesh.getFaceHalfEdges(0);

    const std::vector<int> secondFaceHalfEdges =
        mesh.getFaceHalfEdges(1);

    ASSERT_EQ(firstFaceHalfEdges.size(), 5);
    ASSERT_EQ(secondFaceHalfEdges.size(), 5);

    EXPECT_NE(
        std::find(
            firstFaceHalfEdges.begin(),
            firstFaceHalfEdges.end(),
            result.firstHalfEdgeId
        ),
        firstFaceHalfEdges.end()
    );

    EXPECT_NE(
        std::find(
            firstFaceHalfEdges.begin(),
            firstFaceHalfEdges.end(),
            result.firstNewHalfEdgeId
        ),
        firstFaceHalfEdges.end()
    );

    EXPECT_NE(
        std::find(
            secondFaceHalfEdges.begin(),
            secondFaceHalfEdges.end(),
            result.twinHalfEdgeId
        ),
        secondFaceHalfEdges.end()
    );

    EXPECT_NE(
        std::find(
            secondFaceHalfEdges.begin(),
            secondFaceHalfEdges.end(),
            result.twinNewHalfEdgeId
        ),
        secondFaceHalfEdges.end()
    );
}

TEST(
    HalfEdgeMesh,
    InternalEdgeSplitPreservesTwinSymmetry
)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    const InternalHalfEdgeSplitResult result =
        mesh.splitInternalHalfEdge(
            1,
            Point3{1.0, 0.5, 0.0}
        );

    ASSERT_TRUE(result.success);

    const std::vector<int> splitHalfEdgeIds
    {
        result.firstHalfEdgeId,
        result.firstNewHalfEdgeId,
        result.twinHalfEdgeId,
        result.twinNewHalfEdgeId
    };

    for (int halfEdgeId : splitHalfEdgeIds)
    {
        ASSERT_GE(halfEdgeId, 0);
        ASSERT_LT(
            halfEdgeId,
            static_cast<int>(mesh.halfEdges.size())
        );

        const int twinId =
            mesh.halfEdges[halfEdgeId].twin;

        ASSERT_GE(twinId, 0);
        ASSERT_LT(
            twinId,
            static_cast<int>(mesh.halfEdges.size())
        );

        EXPECT_EQ(
            mesh.halfEdges[twinId].twin,
            halfEdgeId
        );
    }
}

TEST(
    HalfEdgeMesh,
    InternalEdgeSplitLeavesNoBrokenNextLinks
)
{
    HalfEdgeMesh mesh;
    mesh.createTwoQuadMesh();

    const InternalHalfEdgeSplitResult result =
        mesh.splitInternalHalfEdge(
            1,
            Point3{1.0, 0.5, 0.0}
        );

    ASSERT_TRUE(result.success);

    for (int faceId = 0;
         faceId < 2;
         ++faceId)
    {
        const std::vector<int> faceHalfEdges =
            mesh.getFaceHalfEdges(faceId);

        ASSERT_EQ(faceHalfEdges.size(), 5);

        for (int halfEdgeId : faceHalfEdges)
        {
            const int nextId =
                mesh.halfEdges[halfEdgeId].next;

            ASSERT_GE(nextId, 0);

            ASSERT_LT(
                nextId,
                static_cast<int>(
                    mesh.halfEdges.size()
                )
            );

            EXPECT_EQ(
                mesh.halfEdges[nextId].face,
                faceId
            );
        }
    }
}

TEST(HalfEdgeMesh, CreatesValidFourQuadGrid)
{
    HalfEdgeMesh mesh;
    mesh.createFourQuadGrid();

    EXPECT_EQ(mesh.vertices.size(), 9);
    EXPECT_EQ(mesh.faces.size(), 4);
    EXPECT_EQ(mesh.halfEdges.size(), 16);

    for (int faceId = 0; faceId < 4; ++faceId)
    {
        const std::vector<int> faceHalfEdges =
            mesh.getFaceHalfEdges(faceId);

        EXPECT_EQ(faceHalfEdges.size(), 4);
    }

    int halfEdgesWithTwins = 0;

    for (int halfEdgeId = 0;
         halfEdgeId < static_cast<int>(mesh.halfEdges.size());
         ++halfEdgeId)
    {
        const int twinId =
            mesh.halfEdges[halfEdgeId].twin;

        if (twinId < 0)
        {
            continue;
        }

        ++halfEdgesWithTwins;

        ASSERT_LT(
            twinId,
            static_cast<int>(mesh.halfEdges.size())
        );

        EXPECT_EQ(
            mesh.halfEdges[twinId].twin,
            halfEdgeId
        );
    }

    EXPECT_EQ(halfEdgesWithTwins, 8);
}

TEST(
    HalfEdgeMesh,
    FindsOutgoingHalfEdgeInFace
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    EXPECT_EQ(
        mesh.findOutgoingHalfEdgeInFace(
            0,
            0
        ),
        0
    );

    EXPECT_EQ(
        mesh.findOutgoingHalfEdgeInFace(
            0,
            1
        ),
        1
    );

    EXPECT_EQ(
        mesh.findOutgoingHalfEdgeInFace(
            0,
            2
        ),
        2
    );

    EXPECT_EQ(
        mesh.findOutgoingHalfEdgeInFace(
            0,
            3
        ),
        3
    );
}

TEST(
    HalfEdgeMesh,
    FindOutgoingHalfEdgeInFaceReturnsInvalidWhenVertexIsNotInFace
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    EXPECT_EQ(
        mesh.findOutgoingHalfEdgeInFace(
            0,
            99
        ),
        -1
    );
}

TEST(
    HalfEdgeMesh,
    FindsPreviousHalfEdgeInFace
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    EXPECT_EQ(
        mesh.findPreviousHalfEdgeInFace(
            0,
            0
        ),
        3
    );

    EXPECT_EQ(
        mesh.findPreviousHalfEdgeInFace(
            0,
            1
        ),
        0
    );

    EXPECT_EQ(
        mesh.findPreviousHalfEdgeInFace(
            0,
            2
        ),
        1
    );

    EXPECT_EQ(
        mesh.findPreviousHalfEdgeInFace(
            0,
            3
        ),
        2
    );
}

TEST(
    HalfEdgeMesh,
    FindPreviousHalfEdgeInFaceReturnsInvalidForUnknownHalfEdge
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    EXPECT_EQ(
        mesh.findPreviousHalfEdgeInFace(
            0,
            99
        ),
        -1
    );
}

TEST(
    HalfEdgeMesh,
    FindsFaceContainingTwoVertices
)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    EXPECT_EQ(
        mesh.findFaceContainingVertices(
            0,
            2
        ),
        0
    );
}

TEST(
    HalfEdgeMesh,
    CreatesValidTestTriangle
)
{
    HalfEdgeMesh mesh;
    mesh.createTestTriangle();

    ASSERT_EQ(mesh.vertices.size(), 3);
    ASSERT_EQ(mesh.halfEdges.size(), 3);
    ASSERT_EQ(mesh.faces.size(), 1);

    const std::vector<int> faceHalfEdges =
        mesh.getFaceHalfEdges(0);

    ASSERT_EQ(faceHalfEdges.size(), 3);

    EXPECT_EQ(faceHalfEdges[0], 0);
    EXPECT_EQ(faceHalfEdges[1], 1);
    EXPECT_EQ(faceHalfEdges[2], 2);

    EXPECT_EQ(mesh.halfEdges[0].startVertex, 0);
    EXPECT_EQ(mesh.halfEdges[0].endVertex, 1);

    EXPECT_EQ(mesh.halfEdges[1].startVertex, 1);
    EXPECT_EQ(mesh.halfEdges[1].endVertex, 2);

    EXPECT_EQ(mesh.halfEdges[2].startVertex, 2);
    EXPECT_EQ(mesh.halfEdges[2].endVertex, 0);

    EXPECT_EQ(mesh.halfEdges[0].next, 1);
    EXPECT_EQ(mesh.halfEdges[1].next, 2);
    EXPECT_EQ(mesh.halfEdges[2].next, 0);

    EXPECT_EQ(mesh.halfEdges[0].twin, -1);
    EXPECT_EQ(mesh.halfEdges[1].twin, -1);
    EXPECT_EQ(mesh.halfEdges[2].twin, -1);

    EXPECT_EQ(mesh.halfEdges[0].face, 0);
    EXPECT_EQ(mesh.halfEdges[1].face, 0);
    EXPECT_EQ(mesh.halfEdges[2].face, 0);
}

TEST(
    HalfEdgeMesh,
    GetsOutgoingHalfEdgesAtVertex
)
{
    HalfEdgeMesh mesh;
    mesh.createFourQuadGrid();

    int centreVertexId = -1;

    /*
        Find the grid vertex with four outgoing
        half-edges rather than relying on a fixed ID.
    */
    for (int vertexId = 0;
         vertexId <
             static_cast<int>(
                 mesh.vertices.size()
             );
         ++vertexId)
    {
        int outgoingCount = 0;

        for (const HalfEdge& halfEdge :
             mesh.halfEdges)
        {
            if (halfEdge.startVertex ==
                vertexId)
            {
                ++outgoingCount;
            }
        }

        if (outgoingCount == 4)
        {
            centreVertexId =
                vertexId;

            break;
        }
    }

    ASSERT_GE(
        centreVertexId,
        0
    );

    const std::vector<int>
        outgoingHalfEdgeIds =
            mesh.getOutgoingHalfEdgesAtVertex(
                centreVertexId
            );

    ASSERT_EQ(
        outgoingHalfEdgeIds.size(),
        4
    );

    for (int halfEdgeId :
         outgoingHalfEdgeIds)
    {
        ASSERT_GE(
            halfEdgeId,
            0
        );

        ASSERT_LT(
            halfEdgeId,
            static_cast<int>(
                mesh.halfEdges.size()
            )
        );

        EXPECT_EQ(
            mesh.halfEdges[
                halfEdgeId
            ].startVertex,
            centreVertexId
        );
    }
}
