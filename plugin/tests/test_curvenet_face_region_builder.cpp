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
