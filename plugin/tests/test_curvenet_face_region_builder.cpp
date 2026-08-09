#include <gtest/gtest.h>
#include <algorithm>

#include "CurvenetFaceRegionBuilder.h"

TEST(
    CurvenetFaceRegionBuilder,
    StoresSingleMeshFaceInsideCurvenetRegion
)
{
    CurvenetCutResult cutResult;
    cutResult.mesh.createFourQuadGrid();

    /*
        Use the four boundary edges of mesh Face 0:

        vertex 0 ---- vertex 1
           |              |
           |    Face 0    |
           |              |
        vertex 3 ---- vertex 4

        Half-edges:
        0: 0 -> 1
        1: 1 -> 4
        2: 4 -> 3
        3: 3 -> 0
    */
    CutChain topChain;
    topChain.curveId = 0;
    topChain.vertexIds = {
        0,
        1
    };
    topChain.halfEdgeIds = {
        0
    };

    CutChain rightChain;
    rightChain.curveId = 1;
    rightChain.vertexIds = {
        1,
        4
    };
    rightChain.halfEdgeIds = {
        1
    };

    CutChain bottomChain;
    bottomChain.curveId = 2;
    bottomChain.vertexIds = {
        4,
        3
    };
    bottomChain.halfEdgeIds = {
        2
    };

    CutChain leftChain;
    leftChain.curveId = 3;
    leftChain.vertexIds = {
        3,
        0
    };
    leftChain.halfEdgeIds = {
        3
    };

    cutResult.cutChainsByCurveId[0] =
        topChain;

    cutResult.cutChainsByCurveId[1] =
        rightChain;

    cutResult.cutChainsByCurveId[2] =
        bottomChain;

    cutResult.cutChainsByCurveId[3] =
        leftChain;

    CurvenetFace curvenetFace;
    curvenetFace.id = 0;

    CurvenetFaceBoundary topBoundary;
    topBoundary.curveId = 0;
    topBoundary.startVertexId = 0;
    topBoundary.endVertexId = 1;
    topBoundary.reversed = false;

    CurvenetFaceBoundary rightBoundary;
    rightBoundary.curveId = 1;
    rightBoundary.startVertexId = 1;
    rightBoundary.endVertexId = 4;
    rightBoundary.reversed = false;

    CurvenetFaceBoundary bottomBoundary;
    bottomBoundary.curveId = 2;
    bottomBoundary.startVertexId = 4;
    bottomBoundary.endVertexId = 3;
    bottomBoundary.reversed = false;

    CurvenetFaceBoundary leftBoundary;
    leftBoundary.curveId = 3;
    leftBoundary.startVertexId = 3;
    leftBoundary.endVertexId = 0;
    leftBoundary.reversed = false;

    curvenetFace.boundary = {
        topBoundary,
        rightBoundary,
        bottomBoundary,
        leftBoundary
    };

    cutResult.curvenetFaces = {
        curvenetFace
    };

    CurvenetFaceRegionBuilder::build(
        cutResult
    );

    ASSERT_EQ(
        cutResult.curvenetFaces.size(),
        1
    );

    const CurvenetFace& resultFace =
        cutResult.curvenetFaces.front();

    /*
        The four Curvenet boundary chains enclose
        only mesh Face 0.
    */
    ASSERT_EQ(
        resultFace.meshFaceIds.size(),
        1
    );

    EXPECT_EQ(
        resultFace.meshFaceIds.front(),
        0
    );
}

TEST(
    CurvenetFaceRegionBuilder,
    StoresMultipleMeshFacesInsideCurvenetRegion
)
{
    CurvenetCutResult cutResult;
    cutResult.mesh.createFourQuadGrid();

    /*
        Curvenet boundary enclosing the two
        top mesh faces:

        0 -------- 1 -------- 2
        |       Face 0  Face 1 |
        3 -------- 4 -------- 5

        Boundary traversal:
        0 -> 1 -> 2 -> 5 -> 4 -> 3 -> 0

        The internal edge between vertices 1 and 4
        is not part of the Curvenet boundary.
    */
    CutChain topLeftChain;
    topLeftChain.curveId = 0;
    topLeftChain.vertexIds = {
        0,
        1
    };
    topLeftChain.halfEdgeIds = {
        0
    };

    CutChain topRightChain;
    topRightChain.curveId = 1;
    topRightChain.vertexIds = {
        1,
        2
    };
    topRightChain.halfEdgeIds = {
        4
    };

    CutChain rightChain;
    rightChain.curveId = 2;
    rightChain.vertexIds = {
        2,
        5
    };
    rightChain.halfEdgeIds = {
        5
    };

    CutChain bottomRightChain;
    bottomRightChain.curveId = 3;
    bottomRightChain.vertexIds = {
        5,
        4
    };
    bottomRightChain.halfEdgeIds = {
        6
    };

    CutChain bottomLeftChain;
    bottomLeftChain.curveId = 4;
    bottomLeftChain.vertexIds = {
        4,
        3
    };
    bottomLeftChain.halfEdgeIds = {
        2
    };

    CutChain leftChain;
    leftChain.curveId = 5;
    leftChain.vertexIds = {
        3,
        0
    };
    leftChain.halfEdgeIds = {
        3
    };

    cutResult.cutChainsByCurveId[0] =
        topLeftChain;

    cutResult.cutChainsByCurveId[1] =
        topRightChain;

    cutResult.cutChainsByCurveId[2] =
        rightChain;

    cutResult.cutChainsByCurveId[3] =
        bottomRightChain;

    cutResult.cutChainsByCurveId[4] =
        bottomLeftChain;

    cutResult.cutChainsByCurveId[5] =
        leftChain;

    CurvenetFace curvenetFace;
    curvenetFace.id = 0;

    CurvenetFaceBoundary topLeftBoundary;
    topLeftBoundary.curveId = 0;
    topLeftBoundary.startVertexId = 0;
    topLeftBoundary.endVertexId = 1;
    topLeftBoundary.reversed = false;

    CurvenetFaceBoundary topRightBoundary;
    topRightBoundary.curveId = 1;
    topRightBoundary.startVertexId = 1;
    topRightBoundary.endVertexId = 2;
    topRightBoundary.reversed = false;

    CurvenetFaceBoundary rightBoundary;
    rightBoundary.curveId = 2;
    rightBoundary.startVertexId = 2;
    rightBoundary.endVertexId = 5;
    rightBoundary.reversed = false;

    CurvenetFaceBoundary bottomRightBoundary;
    bottomRightBoundary.curveId = 3;
    bottomRightBoundary.startVertexId = 5;
    bottomRightBoundary.endVertexId = 4;
    bottomRightBoundary.reversed = false;

    CurvenetFaceBoundary bottomLeftBoundary;
    bottomLeftBoundary.curveId = 4;
    bottomLeftBoundary.startVertexId = 4;
    bottomLeftBoundary.endVertexId = 3;
    bottomLeftBoundary.reversed = false;

    CurvenetFaceBoundary leftBoundary;
    leftBoundary.curveId = 5;
    leftBoundary.startVertexId = 3;
    leftBoundary.endVertexId = 0;
    leftBoundary.reversed = false;

    curvenetFace.boundary = {
        topLeftBoundary,
        topRightBoundary,
        rightBoundary,
        bottomRightBoundary,
        bottomLeftBoundary,
        leftBoundary
    };

    cutResult.curvenetFaces = {
        curvenetFace
    };

    CurvenetFaceRegionBuilder::build(
        cutResult
    );

    ASSERT_EQ(
        cutResult.curvenetFaces.size(),
        1
    );

    const CurvenetFace& resultFace =
        cutResult.curvenetFaces.front();

    /*
        The flood fill should cross the internal
        edge between Faces 0 and 1, while stopping
        at the outer Curvenet boundary.
    */
    ASSERT_EQ(
        resultFace.meshFaceIds.size(),
        2
    );

    EXPECT_EQ(
        resultFace.meshFaceIds[0],
        0
    );

    EXPECT_EQ(
        resultFace.meshFaceIds[1],
        1
    );
}

TEST(
    CurvenetFaceRegionBuilder,
    KeepsNeighbouringCurvenetFacesSeparate
)
{
    CurvenetCutResult cutResult;
    cutResult.mesh.createFourQuadGrid();

    /*
        Two neighbouring logical Curvenet faces:

        0 -------- 1 -------- 2
        |  Face A  |  Face B  |
        3 -------- 4 -------- 5

        The shared boundary is Curve 1:
        vertex 1 <-> vertex 4.
    */

    CutChain topLeftChain;
    topLeftChain.curveId = 0;
    topLeftChain.vertexIds = {
        0,
        1
    };
    topLeftChain.halfEdgeIds = {
        0
    };

    CutChain sharedChain;
    sharedChain.curveId = 1;
    sharedChain.vertexIds = {
        1,
        4
    };
    sharedChain.halfEdgeIds = {
        1
    };

    CutChain bottomLeftChain;
    bottomLeftChain.curveId = 2;
    bottomLeftChain.vertexIds = {
        4,
        3
    };
    bottomLeftChain.halfEdgeIds = {
        2
    };

    CutChain leftChain;
    leftChain.curveId = 3;
    leftChain.vertexIds = {
        3,
        0
    };
    leftChain.halfEdgeIds = {
        3
    };

    CutChain topRightChain;
    topRightChain.curveId = 4;
    topRightChain.vertexIds = {
        1,
        2
    };
    topRightChain.halfEdgeIds = {
        4
    };

    CutChain rightChain;
    rightChain.curveId = 5;
    rightChain.vertexIds = {
        2,
        5
    };
    rightChain.halfEdgeIds = {
        5
    };

    CutChain bottomRightChain;
    bottomRightChain.curveId = 6;
    bottomRightChain.vertexIds = {
        5,
        4
    };
    bottomRightChain.halfEdgeIds = {
        6
    };

    cutResult.cutChainsByCurveId[0] =
        topLeftChain;

    cutResult.cutChainsByCurveId[1] =
        sharedChain;

    cutResult.cutChainsByCurveId[2] =
        bottomLeftChain;

    cutResult.cutChainsByCurveId[3] =
        leftChain;

    cutResult.cutChainsByCurveId[4] =
        topRightChain;

    cutResult.cutChainsByCurveId[5] =
        rightChain;

    cutResult.cutChainsByCurveId[6] =
        bottomRightChain;

    /*
        Face A boundary:

        0 -> 1 -> 4 -> 3 -> 0
    */
    CurvenetFace faceA;
    faceA.id = 0;

    CurvenetFaceBoundary faceATop;
    faceATop.curveId = 0;
    faceATop.startVertexId = 0;
    faceATop.endVertexId = 1;
    faceATop.reversed = false;

    CurvenetFaceBoundary faceAShared;
    faceAShared.curveId = 1;
    faceAShared.startVertexId = 1;
    faceAShared.endVertexId = 4;
    faceAShared.reversed = false;

    CurvenetFaceBoundary faceABottom;
    faceABottom.curveId = 2;
    faceABottom.startVertexId = 4;
    faceABottom.endVertexId = 3;
    faceABottom.reversed = false;

    CurvenetFaceBoundary faceALeft;
    faceALeft.curveId = 3;
    faceALeft.startVertexId = 3;
    faceALeft.endVertexId = 0;
    faceALeft.reversed = false;

    faceA.boundary = {
        faceATop,
        faceAShared,
        faceABottom,
        faceALeft
    };

    /*
        Face B boundary:

        1 -> 2 -> 5 -> 4 -> 1

        The shared chain is stored as 1 -> 4,
        so Face B traverses it in reverse.
    */
    CurvenetFace faceB;
    faceB.id = 1;

    CurvenetFaceBoundary faceBTop;
    faceBTop.curveId = 4;
    faceBTop.startVertexId = 1;
    faceBTop.endVertexId = 2;
    faceBTop.reversed = false;

    CurvenetFaceBoundary faceBRight;
    faceBRight.curveId = 5;
    faceBRight.startVertexId = 2;
    faceBRight.endVertexId = 5;
    faceBRight.reversed = false;

    CurvenetFaceBoundary faceBBottom;
    faceBBottom.curveId = 6;
    faceBBottom.startVertexId = 5;
    faceBBottom.endVertexId = 4;
    faceBBottom.reversed = false;

    CurvenetFaceBoundary faceBShared;
    faceBShared.curveId = 1;
    faceBShared.startVertexId = 4;
    faceBShared.endVertexId = 1;
    faceBShared.reversed = true;

    faceB.boundary = {
        faceBTop,
        faceBRight,
        faceBBottom,
        faceBShared
    };

    cutResult.curvenetFaces = {
        faceA,
        faceB
    };

    CurvenetFaceRegionBuilder::build(
        cutResult
    );

    ASSERT_EQ(
        cutResult.curvenetFaces.size(),
        2
    );

    const CurvenetFace& resultFaceA =
        cutResult.curvenetFaces[0];

    const CurvenetFace& resultFaceB =
        cutResult.curvenetFaces[1];

    /*
        Each logical Curvenet face should contain
        its corresponding mesh polygon face only.
    */
    ASSERT_EQ(
        resultFaceA.meshFaceIds.size(),
        1
    );

    EXPECT_EQ(
        resultFaceA.meshFaceIds.front(),
        0
    );

    ASSERT_EQ(
        resultFaceB.meshFaceIds.size(),
        1
    );

    EXPECT_EQ(
        resultFaceB.meshFaceIds.front(),
        1
    );

    /*
        Both Curvenet faces should reference
        Curve 1 as their shared boundary.
    */
    EXPECT_EQ(
        std::count_if(
            resultFaceA.boundary.begin(),
            resultFaceA.boundary.end(),
            [](
                const CurvenetFaceBoundary& boundary
            )
            {
                return boundary.curveId == 1;
            }
        ),
        1
    );

    EXPECT_EQ(
        std::count_if(
            resultFaceB.boundary.begin(),
            resultFaceB.boundary.end(),
            [](
                const CurvenetFaceBoundary& boundary
            )
            {
                return boundary.curveId == 1;
            }
        ),
        1
    );

    /*
        The two regions must remain separate.
    */
    EXPECT_NE(
        resultFaceA.meshFaceIds.front(),
        resultFaceB.meshFaceIds.front()
    );
}

TEST(
    CurvenetFaceRegionBuilder,
    UsesTwinFaceAsRegionSeed
)
{
    CurvenetCutResult cutResult;
    cutResult.mesh.createFourQuadGrid();

    /*
        Mesh Face 0 has these directed boundary edges:

        0: 0 -> 1
        1: 1 -> 4
        2: 4 -> 3
        3: 3 -> 0

        Traverse the same boundary in reverse:

        4 -> 1 -> 0 -> 3 -> 4

        The first edge, 4 -> 1, is the twin of
        Face 0's edge 1 -> 4. Its owning face is the
        neighbouring mesh face, while its twin owns
        the intended region, Face 0.
    */

    const auto getOrCreateTwin =
        [&cutResult](
            int halfEdgeId
        ) -> int
        {
            if (
                halfEdgeId < 0 ||
                halfEdgeId >=
                    static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    )
            )
            {
                return -1;
            }

            const int existingTwinId =
                cutResult.mesh.halfEdges[
                    halfEdgeId
                ].twin;

            if (existingTwinId >= 0)
            {
                return existingTwinId;
            }

            const HalfEdge& sourceHalfEdge =
                cutResult.mesh.halfEdges[
                    halfEdgeId
                ];

            HalfEdge reverseHalfEdge;

            reverseHalfEdge.startVertex =
                sourceHalfEdge.endVertex;

            reverseHalfEdge.endVertex =
                sourceHalfEdge.startVertex;

            reverseHalfEdge.face = -1;
            reverseHalfEdge.next = -1;
            reverseHalfEdge.twin =
                halfEdgeId;

            const int reverseHalfEdgeId =
                static_cast<int>(
                    cutResult.mesh.halfEdges.size()
                );

            cutResult.mesh.halfEdges.push_back(
                reverseHalfEdge
            );

            cutResult.mesh.halfEdges[
                halfEdgeId
            ].twin =
                reverseHalfEdgeId;

            return reverseHalfEdgeId;
        };

    const int edge4To1 =
        getOrCreateTwin(1);

    const int edge1To0 =
        getOrCreateTwin(0);

    const int edge0To3 =
        getOrCreateTwin(3);

    const int edge3To4 =
        getOrCreateTwin(2);

    ASSERT_GE(
        edge4To1,
        0
    );

    ASSERT_GE(
        edge1To0,
        0
    );

    ASSERT_GE(
        edge0To3,
        0
    );

    ASSERT_GE(
        edge3To4,
        0
    );

    /*
        Confirm that the first oriented edge owns the
        neighbouring face and its twin owns Face 0.
    */
    ASSERT_NE(
        cutResult.mesh.halfEdges[
            edge4To1
        ].face,
        0
    );

    ASSERT_EQ(
        cutResult.mesh.halfEdges[
            cutResult.mesh.halfEdges[
                edge4To1
            ].twin
        ].face,
        0
    );

    CutChain firstChain;
    firstChain.curveId = 0;
    firstChain.vertexIds = {
        4,
        1
    };
    firstChain.halfEdgeIds = {
        edge4To1
    };

    CutChain secondChain;
    secondChain.curveId = 1;
    secondChain.vertexIds = {
        1,
        0
    };
    secondChain.halfEdgeIds = {
        edge1To0
    };

    CutChain thirdChain;
    thirdChain.curveId = 2;
    thirdChain.vertexIds = {
        0,
        3
    };
    thirdChain.halfEdgeIds = {
        edge0To3
    };

    CutChain fourthChain;
    fourthChain.curveId = 3;
    fourthChain.vertexIds = {
        3,
        4
    };
    fourthChain.halfEdgeIds = {
        edge3To4
    };

    cutResult.cutChainsByCurveId[0] =
        firstChain;

    cutResult.cutChainsByCurveId[1] =
        secondChain;

    cutResult.cutChainsByCurveId[2] =
        thirdChain;

    cutResult.cutChainsByCurveId[3] =
        fourthChain;

    CurvenetFace curvenetFace;
    curvenetFace.id = 0;

    CurvenetFaceBoundary firstBoundary;
    firstBoundary.curveId = 0;
    firstBoundary.startVertexId = 4;
    firstBoundary.endVertexId = 1;
    firstBoundary.reversed = false;

    CurvenetFaceBoundary secondBoundary;
    secondBoundary.curveId = 1;
    secondBoundary.startVertexId = 1;
    secondBoundary.endVertexId = 0;
    secondBoundary.reversed = false;

    CurvenetFaceBoundary thirdBoundary;
    thirdBoundary.curveId = 2;
    thirdBoundary.startVertexId = 0;
    thirdBoundary.endVertexId = 3;
    thirdBoundary.reversed = false;

    CurvenetFaceBoundary fourthBoundary;
    fourthBoundary.curveId = 3;
    fourthBoundary.startVertexId = 3;
    fourthBoundary.endVertexId = 4;
    fourthBoundary.reversed = false;

    curvenetFace.boundary = {
        firstBoundary,
        secondBoundary,
        thirdBoundary,
        fourthBoundary
    };

    cutResult.curvenetFaces = {
        curvenetFace
    };

    CurvenetFaceRegionBuilder::build(
        cutResult
    );

    ASSERT_EQ(
        cutResult.curvenetFaces.size(),
        1
    );

    const CurvenetFace& resultFace =
        cutResult.curvenetFaces.front();

    ASSERT_EQ(
        resultFace.meshFaceIds.size(),
        1
    );

    EXPECT_EQ(
        resultFace.meshFaceIds.front(),
        0
    );
}

TEST(
    CurvenetFaceRegionBuilder,
    BuildsEveryComponentForFullSurfaceCurvenet
)
{
    CurvenetCutResult cutResult;
    cutResult.mesh.createFourQuadGrid();

    for (int halfEdgeId = 0;
         halfEdgeId < static_cast<int>(cutResult.mesh.halfEdges.size());
         ++halfEdgeId)
    {
        const HalfEdge& halfEdge =
            cutResult.mesh.halfEdges[halfEdgeId];

        const bool upperDivider =
            (halfEdge.startVertex == 1 && halfEdge.endVertex == 4) ||
            (halfEdge.startVertex == 4 && halfEdge.endVertex == 1);

        const bool lowerDivider =
            (halfEdge.startVertex == 4 && halfEdge.endVertex == 7) ||
            (halfEdge.startVertex == 7 && halfEdge.endVertex == 4);

        if (upperDivider || lowerDivider)
        {
            cutResult.embeddedHalfEdgeIds.insert(halfEdgeId);
        }
    }

    CurvenetFaceRegionBuilder::buildFullSurfacePartitions(cutResult);

    ASSERT_EQ(cutResult.curvenetFaces.size(), 2);
    EXPECT_EQ(cutResult.curvenetFaces[0].meshFaceIds.size(), 2);
    EXPECT_EQ(cutResult.curvenetFaces[1].meshFaceIds.size(), 2);
}

TEST(
    CurvenetFaceRegionBuilder,
    MergesNearZeroAreaFullSurfacePartition
)
{
    CurvenetCutResult cutResult;
    constexpr double sliverWidth = 0.00005;

    cutResult.mesh.vertices = {
        Vertex{Point3{0.0, 0.0, 0.0}},
        Vertex{Point3{sliverWidth, 0.0, 0.0}},
        Vertex{Point3{1.0 + sliverWidth, 0.0, 0.0}},
        Vertex{Point3{0.0, 1.0, 0.0}},
        Vertex{Point3{sliverWidth, 1.0, 0.0}},
        Vertex{Point3{1.0 + sliverWidth, 1.0, 0.0}}
    };

    cutResult.mesh.halfEdges = {
        HalfEdge{0, 1, 1, -1, 0},
        HalfEdge{1, 4, 2, -1, 0},
        HalfEdge{4, 3, 3, -1, 0},
        HalfEdge{3, 0, 0, -1, 0},
        HalfEdge{1, 2, 5, -1, 1},
        HalfEdge{2, 5, 6, -1, 1},
        HalfEdge{5, 4, 7, -1, 1},
        HalfEdge{4, 1, 4, -1, 1}
    };
    cutResult.mesh.faces = {
        Face{0},
        Face{4}
    };
    cutResult.mesh.assignTwins();

    /*
        Treat the shared edge as an embedded curve. The left
        component is a numerical sliver and should be absorbed
        into its normal-area neighbour.
    */
    cutResult.embeddedHalfEdgeIds.insert(1);
    cutResult.embeddedHalfEdgeIds.insert(7);

    CurvenetFaceRegionBuilder::buildFullSurfacePartitions(cutResult);

    ASSERT_EQ(cutResult.curvenetFaces.size(), 1);
    EXPECT_EQ(cutResult.curvenetFaces[0].meshFaceIds.size(), 2);
    EXPECT_EQ(cutResult.curvenetFaces[0].meshFaceIds[0], 0);
    EXPECT_EQ(cutResult.curvenetFaces[0].meshFaceIds[1], 1);
}
