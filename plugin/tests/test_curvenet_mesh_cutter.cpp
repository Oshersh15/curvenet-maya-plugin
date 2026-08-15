/* Tests multi-profile cutting, connections, failures, and evolving topology. */

#include <gtest/gtest.h>
#include <algorithm>

#include "CurvenetMeshCutter.h"

TEST(
    CurvenetMeshCutter,
    ProcessesMultipleCutPathsOnOneEvolvingMesh
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    /*
        Left vertical path:

        (0.5, 2.0)
             ↓
        (0.5, 1.0)
             ↓
        (0.5, 0.0)
    */
    CutPath leftPath;
    leftPath.curveId = 0;
    leftPath.closed = false;

    CutVertex leftTop;
    leftTop.position =
        Point3{0.5, 2.0, 0.0};
    leftTop.sourceHalfEdgeId = 0;
    leftTop.sourceEdgeT = 0.5;
    leftTop.curveId = 0;
    leftTop.cutPathOrder = 0;

    CutVertex leftMiddle;
    leftMiddle.position =
        Point3{0.5, 1.0, 0.0};
    leftMiddle.sourceHalfEdgeId = 2;
    leftMiddle.sourceEdgeT = 0.5;
    leftMiddle.curveId = 0;
    leftMiddle.cutPathOrder = 1;

    CutVertex leftBottom;
    leftBottom.position =
        Point3{0.5, 0.0, 0.0};
    leftBottom.sourceHalfEdgeId = 10;
    leftBottom.sourceEdgeT = 0.5;
    leftBottom.curveId = 0;
    leftBottom.cutPathOrder = 2;

    leftPath.cutVertices = {
        leftTop,
        leftMiddle,
        leftBottom
    };

    leftPath.faceIntervalIds = {
        0,
        2
    };

    /*
        Right vertical path:

        (1.5, 2.0)
             ↓
        (1.5, 1.0)
             ↓
        (1.5, 0.0)
    */
    CutPath rightPath;
    rightPath.curveId = 1;
    rightPath.closed = false;

    CutVertex rightTop;
    rightTop.position =
        Point3{1.5, 2.0, 0.0};
    rightTop.sourceHalfEdgeId = 4;
    rightTop.sourceEdgeT = 0.5;
    rightTop.curveId = 1;
    rightTop.cutPathOrder = 0;

    CutVertex rightMiddle;
    rightMiddle.position =
        Point3{1.5, 1.0, 0.0};
    rightMiddle.sourceHalfEdgeId = 6;
    rightMiddle.sourceEdgeT = 0.5;
    rightMiddle.curveId = 1;
    rightMiddle.cutPathOrder = 1;

    CutVertex rightBottom;
    rightBottom.position =
        Point3{1.5, 0.0, 0.0};
    rightBottom.sourceHalfEdgeId = 14;
    rightBottom.sourceEdgeT = 0.5;
    rightBottom.curveId = 1;
    rightBottom.cutPathOrder = 2;

    rightPath.cutVertices = {
        rightTop,
        rightMiddle,
        rightBottom
    };

    rightPath.faceIntervalIds = {
        1,
        3
    };

    const std::vector<CutPath> cutPaths = {
        leftPath,
        rightPath
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            cutPaths,
            0.0001
        );

    const CutChain& leftChain =
        result.cutChainsByCurveId.at(0);

    const CutChain& rightChain =
        result.cutChainsByCurveId.at(1);

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.profileResults.size(),
        2
    );

    ASSERT_EQ(
        result.cutChainsByCurveId.size(),
        2
    );

    ASSERT_TRUE(
        result.cutChainsByCurveId.find(0) !=
        result.cutChainsByCurveId.end()
    );

    ASSERT_TRUE(
        result.cutChainsByCurveId.find(1) !=
        result.cutChainsByCurveId.end()
    );

    EXPECT_EQ(
        result.profileResults[0].cutChain.curveId,
        0
    );

    EXPECT_EQ(
        result.profileResults[1].cutChain.curveId,
        1
    );

    EXPECT_EQ(
        result.profileResults[0].cutChain.vertexIds.size(),
        3
    );

    EXPECT_EQ(
        result.profileResults[1].cutChain.vertexIds.size(),
        3
    );

    EXPECT_EQ(
        result.profileResults[0].cutChain.halfEdgeIds.size(),
        2
    );

    EXPECT_EQ(
        result.profileResults[1].cutChain.halfEdgeIds.size(),
        2
    );

    EXPECT_FALSE(
        result.profileResults[0].cutChain.closed
    );

    EXPECT_FALSE(
        result.profileResults[1].cutChain.closed
    );

    /*
        Both paths must have modified the same result mesh.
    */
    EXPECT_EQ(result.mesh.vertices.size(), 15);
    EXPECT_EQ(result.mesh.halfEdges.size(), 32);
    EXPECT_EQ(result.mesh.faces.size(), 8);

    /*
        The caller's original mesh must remain unchanged.
    */
    EXPECT_EQ(inputMesh.vertices.size(), 9);
    EXPECT_EQ(inputMesh.halfEdges.size(), 16);
    EXPECT_EQ(inputMesh.faces.size(), 4);

    /*
        Each profile curve should have its own
        CutChain accessible through its curve ID.
    */
    EXPECT_EQ(leftChain.curveId, 0);
    EXPECT_EQ(rightChain.curveId, 1);

    EXPECT_EQ(leftChain.vertexIds.size(), 3);
    EXPECT_EQ(rightChain.vertexIds.size(), 3);

    EXPECT_EQ(leftChain.halfEdgeIds.size(), 2);
    EXPECT_EQ(rightChain.halfEdgeIds.size(), 2);

    EXPECT_FALSE(leftChain.closed);
    EXPECT_FALSE(rightChain.closed);
}

TEST(
    CurvenetMeshCutter,
    RejectsDuplicateCurveIds
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    CutPath firstPath;
    firstPath.curveId = 0;
    firstPath.closed = false;

    CutVertex firstTop;
    firstTop.position =
        Point3{0.5, 2.0, 0.0};
    firstTop.sourceHalfEdgeId = 0;
    firstTop.sourceEdgeT = 0.5;
    firstTop.curveId = 0;
    firstTop.cutPathOrder = 0;

    CutVertex firstMiddle;
    firstMiddle.position =
        Point3{0.5, 1.0, 0.0};
    firstMiddle.sourceHalfEdgeId = 2;
    firstMiddle.sourceEdgeT = 0.5;
    firstMiddle.curveId = 0;
    firstMiddle.cutPathOrder = 1;

    CutVertex firstBottom;
    firstBottom.position =
        Point3{0.5, 0.0, 0.0};
    firstBottom.sourceHalfEdgeId = 10;
    firstBottom.sourceEdgeT = 0.5;
    firstBottom.curveId = 0;
    firstBottom.cutPathOrder = 2;

    firstPath.cutVertices = {
        firstTop,
        firstMiddle,
        firstBottom
    };

    firstPath.faceIntervalIds = {
        0,
        2
    };

    /*
        Second CutPath deliberately reuses
        the same logical curve ID.
    */
    CutPath secondPath = firstPath;

    const std::vector<CutPath> cutPaths = {
        firstPath,
        secondPath
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            cutPaths,
            0.0001
        );

    /*
        Duplicate logical curve IDs should
        be rejected.
    */
    EXPECT_FALSE(result.success);

    /*
        Only the first profile should have
        been preserved.
    */
    EXPECT_EQ(
        result.profileResults.size(),
        1
    );

    EXPECT_EQ(
        result.cutChainsByCurveId.size(),
        1
    );

    EXPECT_TRUE(
        result.cutChainsByCurveId.find(0) !=
        result.cutChainsByCurveId.end()
    );
}

TEST(
    CurvenetMeshCutter,
    ReusesSharedEndpointBetweenTwoCutPaths
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    /*
        First profile:

        top boundary
            |
            |
        shared node
    */
    CutPath firstPath;
    firstPath.curveId = 0;
    firstPath.closed = false;

    CutVertex firstTop;
    firstTop.position =
        Point3{0.5, 2.0, 0.0};
    firstTop.sourceHalfEdgeId = 0;
    firstTop.sourceEdgeT = 0.5;
    firstTop.curveId = 0;
    firstTop.cutPathOrder = 0;

    CutVertex firstSharedEndpoint;
    firstSharedEndpoint.position =
        Point3{0.5, 1.0, 0.0};
    firstSharedEndpoint.sourceHalfEdgeId = 2;
    firstSharedEndpoint.sourceEdgeT = 0.5;
    firstSharedEndpoint.curveId = 0;
    firstSharedEndpoint.cutPathOrder = 1;

    firstPath.cutVertices = {
        firstTop,
        firstSharedEndpoint
    };

    firstPath.faceIntervalIds = {
        0
    };

    /*
        Second profile begins at the same Curvenet node
        and travels towards the left boundary.
    */
    CutPath secondPath;
    secondPath.curveId = 1;
    secondPath.closed = false;

    CutVertex secondSharedEndpoint;
    secondSharedEndpoint.position =
        Point3{0.5, 1.0, 0.0};
    secondSharedEndpoint.sourceHalfEdgeId = 2;
    secondSharedEndpoint.sourceEdgeT = 0.5;
    secondSharedEndpoint.curveId = 1;
    secondSharedEndpoint.cutPathOrder = 0;

    CutVertex secondLeftEndpoint;
    secondLeftEndpoint.position =
        Point3{0.0, 1.5, 0.0};
    secondLeftEndpoint.sourceHalfEdgeId = 3;
    secondLeftEndpoint.sourceEdgeT = 0.5;
    secondLeftEndpoint.curveId = 1;
    secondLeftEndpoint.cutPathOrder = 1;

    secondPath.cutVertices = {
        secondSharedEndpoint,
        secondLeftEndpoint
    };

    secondPath.faceIntervalIds = {
        0
    };

    const std::vector<CutPath> cutPaths = {
        firstPath,
        secondPath
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            cutPaths,
            0.0001
        );

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.cutChainsByCurveId.size(),
        2
    );

    const CutChain& firstChain =
        result.cutChainsByCurveId.at(0);

    const CutChain& secondChain =
        result.cutChainsByCurveId.at(1);

    ASSERT_EQ(firstChain.vertexIds.size(), 2);
    ASSERT_EQ(secondChain.vertexIds.size(), 2);

    /*
        The final vertex of Curve 0 and the first
        vertex of Curve 1 must be the same mesh vertex.
    */
    EXPECT_EQ(
        firstChain.vertexIds.back(),
        secondChain.vertexIds.front()
    );

    /*
        Curve 0 creates two mesh vertices.
        Curve 1 reuses the shared one and creates
        only its other endpoint.
    */
    EXPECT_EQ(
        result.mesh.vertices.size(),
        inputMesh.vertices.size() + 3
    );
}

TEST(
    CurvenetMeshCutter,
    ReusesInteriorVertexOfExistingCutChain
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    /*
        First profile creates a vertical chain
        with an interior CutVertex at (0.5, 1.0).
    */
    CutPath verticalPath;
    verticalPath.curveId = 0;
    verticalPath.closed = false;

    CutVertex topCut;
    topCut.position =
        Point3{0.5, 2.0, 0.0};
    topCut.sourceHalfEdgeId = 0;
    topCut.sourceEdgeT = 0.5;
    topCut.curveId = 0;
    topCut.cutPathOrder = 0;

    CutVertex middleCut;
    middleCut.position =
        Point3{0.5, 1.0, 0.0};
    middleCut.sourceHalfEdgeId = 2;
    middleCut.sourceEdgeT = 0.5;
    middleCut.curveId = 0;
    middleCut.cutPathOrder = 1;

    CutVertex bottomCut;
    bottomCut.position =
        Point3{0.5, 0.0, 0.0};
    bottomCut.sourceHalfEdgeId = 10;
    bottomCut.sourceEdgeT = 0.5;
    bottomCut.curveId = 0;
    bottomCut.cutPathOrder = 2;

    verticalPath.cutVertices = {
        topCut,
        middleCut,
        bottomCut
    };

    verticalPath.faceIntervalIds = {
        0,
        2
    };

    /*
        The second profile starts at the interior
        vertex of the first chain and travels left.
    */
    CutPath branchPath;
    branchPath.curveId = 1;
    branchPath.closed = false;

    CutVertex sharedEndpoint;
    sharedEndpoint.position =
        Point3{0.5, 1.0, 0.0};
    sharedEndpoint.sourceHalfEdgeId = 2;
    sharedEndpoint.sourceEdgeT = 0.5;
    sharedEndpoint.curveId = 1;
    sharedEndpoint.cutPathOrder = 0;

    CutVertex leftEndpoint;
    leftEndpoint.position =
        Point3{0.0, 1.5, 0.0};
    leftEndpoint.sourceHalfEdgeId = 3;
    leftEndpoint.sourceEdgeT = 0.5;
    leftEndpoint.curveId = 1;
    leftEndpoint.cutPathOrder = 1;

    branchPath.cutVertices = {
        sharedEndpoint,
        leftEndpoint
    };

    branchPath.faceIntervalIds = {
        0
    };

    const std::vector<CutPath> cutPaths = {
        verticalPath,
        branchPath
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            cutPaths,
            0.0001
        );

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.cutChainsByCurveId.size(),
        2
    );

    const CutChain& verticalChain =
        result.cutChainsByCurveId.at(0);

    const CutChain& branchChain =
        result.cutChainsByCurveId.at(1);

    ASSERT_EQ(
        verticalChain.vertexIds.size(),
        3
    );

    ASSERT_EQ(
        branchChain.vertexIds.size(),
        2
    );

    /*
        The first vertex of the branch must reuse
        the interior vertex of the vertical chain.
    */
    EXPECT_EQ(
        branchChain.vertexIds.front(),
        verticalChain.vertexIds[1]
    );

    /*
        The vertical profile creates three vertices.
        The branch reuses one and creates only its
        outer endpoint.
    */
    EXPECT_EQ(
        result.mesh.vertices.size(),
        inputMesh.vertices.size() + 4
    );
}

TEST(
    CurvenetMeshCutter,
    PreservesOpenAndClosedChains
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    /*
        Closed triangular CutPath.
    */
    CutPath closedPath;
    closedPath.curveId = 0;
    closedPath.closed = true;

    CutVertex closedA;
    closedA.position =
        Point3{0.5, 2.0, 0.0};
    closedA.sourceHalfEdgeId = 0;
    closedA.sourceEdgeT = 0.5;
    closedA.curveId = 0;
    closedA.cutPathOrder = 0;

    CutVertex closedB;
    closedB.position =
        Point3{1.0, 1.5, 0.0};
    closedB.sourceHalfEdgeId = 1;
    closedB.sourceEdgeT = 0.5;
    closedB.curveId = 0;
    closedB.cutPathOrder = 1;

    CutVertex closedC;
    closedC.position =
        Point3{0.5, 1.0, 0.0};
    closedC.sourceHalfEdgeId = 2;
    closedC.sourceEdgeT = 0.5;
    closedC.curveId = 0;
    closedC.cutPathOrder = 2;

    closedPath.cutVertices = {
        closedA,
        closedB,
        closedC
    };

    closedPath.faceIntervalIds = {
        0,
        1,
        0
    };

    /*
        Independent open CutPath.
    */
    CutPath openPath;
    openPath.curveId = 1;
    openPath.closed = false;

    CutVertex openA;
    openA.position =
        Point3{1.5, 2.0, 0.0};
    openA.sourceHalfEdgeId = 4;
    openA.sourceEdgeT = 0.5;
    openA.curveId = 1;
    openA.cutPathOrder = 0;

    CutVertex openB;
    openB.position =
        Point3{1.5, 1.0, 0.0};
    openB.sourceHalfEdgeId = 6;
    openB.sourceEdgeT = 0.5;
    openB.curveId = 1;
    openB.cutPathOrder = 1;

    openPath.cutVertices = {
        openA,
        openB
    };

    openPath.faceIntervalIds = {
        1
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {
                closedPath,
                openPath
            },
            0.0001
        );

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.cutChainsByCurveId.size(),
        2
    );

    const CutChain& closedChain =
        result.cutChainsByCurveId.at(0);

    const CutChain& openChain =
        result.cutChainsByCurveId.at(1);

    /*
        Each CutChain should preserve its
        original open/closed state.
    */
    EXPECT_TRUE(
        closedChain.closed
    );

    EXPECT_FALSE(
        openChain.closed
    );
}

TEST(
    CurvenetMeshCutter,
    StoresEmbeddedMeshVertexIds
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    CutPath verticalPath;
    verticalPath.curveId = 0;
    verticalPath.closed = false;

    CutVertex topCut;
    topCut.position =
        Point3{0.5, 2.0, 0.0};
    topCut.sourceHalfEdgeId = 0;
    topCut.sourceEdgeT = 0.5;
    topCut.curveId = 0;
    topCut.cutPathOrder = 0;

    CutVertex middleCut;
    middleCut.position =
        Point3{0.5, 1.0, 0.0};
    middleCut.sourceHalfEdgeId = 2;
    middleCut.sourceEdgeT = 0.5;
    middleCut.curveId = 0;
    middleCut.cutPathOrder = 1;

    CutVertex bottomCut;
    bottomCut.position =
        Point3{0.5, 0.0, 0.0};
    bottomCut.sourceHalfEdgeId = 10;
    bottomCut.sourceEdgeT = 0.5;
    bottomCut.curveId = 0;
    bottomCut.cutPathOrder = 2;

    verticalPath.cutVertices = {
        topCut,
        middleCut,
        bottomCut
    };

    verticalPath.faceIntervalIds = {
        0,
        2
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {
                verticalPath
            },
            0.0001
        );

    ASSERT_TRUE(result.success);

    /*
        Every vertex belonging to the CutChain
        should also appear in the embedded
        Curvenet vertex set.
    */
    ASSERT_EQ(
        result.embeddedVertexIds.size(),
        result.cutChainsByCurveId
            .at(0)
            .vertexIds
            .size()
    );

    for (int meshVertexId :
         result.cutChainsByCurveId
             .at(0)
             .vertexIds)
    {
        EXPECT_TRUE(
            result.embeddedVertexIds.count(
                meshVertexId
            )
        );
    }
}

TEST(
    CurvenetMeshCutter,
    StoresEmbeddedCutHalfEdgeIds
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    CutPath verticalPath;
    verticalPath.curveId = 0;
    verticalPath.closed = false;

    CutVertex topCut;
    topCut.position =
        Point3{0.5, 2.0, 0.0};
    topCut.sourceHalfEdgeId = 0;
    topCut.sourceEdgeT = 0.5;
    topCut.curveId = 0;
    topCut.cutPathOrder = 0;

    CutVertex middleCut;
    middleCut.position =
        Point3{0.5, 1.0, 0.0};
    middleCut.sourceHalfEdgeId = 2;
    middleCut.sourceEdgeT = 0.5;
    middleCut.curveId = 0;
    middleCut.cutPathOrder = 1;

    CutVertex bottomCut;
    bottomCut.position =
        Point3{0.5, 0.0, 0.0};
    bottomCut.sourceHalfEdgeId = 10;
    bottomCut.sourceEdgeT = 0.5;
    bottomCut.curveId = 0;
    bottomCut.cutPathOrder = 2;

    verticalPath.cutVertices = {
        topCut,
        middleCut,
        bottomCut
    };

    verticalPath.faceIntervalIds = {
        0,
        2
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {
                verticalPath
            },
            0.0001
        );

    ASSERT_TRUE(result.success);

    const CutChain& cutChain =
        result.cutChainsByCurveId.at(0);

    /*
        Every forward half-edge belonging to the
        CutChain should appear in the complete
        embedded Curvenet half-edge set.
    */
    ASSERT_EQ(
        result.embeddedHalfEdgeIds.size(),
        cutChain.halfEdgeIds.size()
    );

    for (int halfEdgeId :
         cutChain.halfEdgeIds)
    {
        EXPECT_TRUE(
            result.embeddedHalfEdgeIds.count(
                halfEdgeId
            )
        );
    }
}

TEST(
    CurvenetMeshCutter,
    StoresAffectedMeshFaceIds
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    CutPath verticalPath;
    verticalPath.curveId = 0;
    verticalPath.closed = false;

    CutVertex topCut;
    topCut.position =
        Point3{0.5, 2.0, 0.0};
    topCut.sourceHalfEdgeId = 0;
    topCut.sourceEdgeT = 0.5;
    topCut.curveId = 0;
    topCut.cutPathOrder = 0;

    CutVertex middleCut;
    middleCut.position =
        Point3{0.5, 1.0, 0.0};
    middleCut.sourceHalfEdgeId = 2;
    middleCut.sourceEdgeT = 0.5;
    middleCut.curveId = 0;
    middleCut.cutPathOrder = 1;

    CutVertex bottomCut;
    bottomCut.position =
        Point3{0.5, 0.0, 0.0};
    bottomCut.sourceHalfEdgeId = 10;
    bottomCut.sourceEdgeT = 0.5;
    bottomCut.curveId = 0;
    bottomCut.cutPathOrder = 2;

    verticalPath.cutVertices = {
        topCut,
        middleCut,
        bottomCut
    };

    verticalPath.faceIntervalIds = {
        0,
        2
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {
                verticalPath
            },
            0.0001
        );

    ASSERT_TRUE(result.success);

    const CutChain& cutChain =
        result.cutChainsByCurveId.at(0);

    ASSERT_FALSE(
        result.embeddedFaceIds.empty()
    );

    /*
        Both faces adjacent to every embedded cut edge
        should appear in the complete Curvenet face set.
    */
    for (int halfEdgeId :
         cutChain.halfEdgeIds)
    {
        ASSERT_GE(halfEdgeId, 0);

        ASSERT_LT(
            halfEdgeId,
            static_cast<int>(
                result.mesh.halfEdges.size()
            )
        );

        const HalfEdge& halfEdge =
            result.mesh.halfEdges[
                halfEdgeId
            ];

        if (halfEdge.face >= 0)
        {
            EXPECT_TRUE(
                result.embeddedFaceIds.count(
                    halfEdge.face
                )
            );
        }

        if (halfEdge.twin >= 0)
        {
            const int twinFaceId =
                result.mesh.halfEdges[
                    halfEdge.twin
                ].face;

            if (twinFaceId >= 0)
            {
                EXPECT_TRUE(
                    result.embeddedFaceIds.count(
                        twinFaceId
                    )
                );
            }
        }
    }
}

TEST(
    CurvenetMeshCutter,
    StoresSharedCurvenetNodeConnectivity
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    /*
        Vertical profile with a shared interior vertex.
    */
    CutPath verticalPath;
    verticalPath.curveId = 0;
    verticalPath.closed = false;

    CutVertex topCut;
    topCut.position =
        Point3{0.5, 2.0, 0.0};
    topCut.sourceHalfEdgeId = 0;
    topCut.sourceEdgeT = 0.5;
    topCut.curveId = 0;
    topCut.cutPathOrder = 0;

    CutVertex middleCut;
    middleCut.position =
        Point3{0.5, 1.0, 0.0};
    middleCut.sourceHalfEdgeId = 2;
    middleCut.sourceEdgeT = 0.5;
    middleCut.curveId = 0;
    middleCut.cutPathOrder = 1;

    CutVertex bottomCut;
    bottomCut.position =
        Point3{0.5, 0.0, 0.0};
    bottomCut.sourceHalfEdgeId = 10;
    bottomCut.sourceEdgeT = 0.5;
    bottomCut.curveId = 0;
    bottomCut.cutPathOrder = 2;

    verticalPath.cutVertices = {
        topCut,
        middleCut,
        bottomCut
    };

    verticalPath.faceIntervalIds = {
        0,
        2
    };

    /*
        Branch profile meeting the vertical profile
        at its interior Curvenet node.
    */
    CutPath branchPath;
    branchPath.curveId = 1;
    branchPath.closed = false;

    CutVertex sharedEndpoint;
    sharedEndpoint.position =
        Point3{0.5, 1.0, 0.0};
    sharedEndpoint.sourceHalfEdgeId = 2;
    sharedEndpoint.sourceEdgeT = 0.5;
    sharedEndpoint.curveId = 1;
    sharedEndpoint.cutPathOrder = 0;

    CutVertex outerEndpoint;
    outerEndpoint.position =
        Point3{0.0, 1.5, 0.0};
    outerEndpoint.sourceHalfEdgeId = 3;
    outerEndpoint.sourceEdgeT = 0.5;
    outerEndpoint.curveId = 1;
    outerEndpoint.cutPathOrder = 1;

    branchPath.cutVertices = {
        sharedEndpoint,
        outerEndpoint
    };

    branchPath.faceIntervalIds = {
        0
    };

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {
                verticalPath,
                branchPath
            },
            0.0001
        );

    ASSERT_TRUE(result.success);

    ASSERT_EQ(
        result.sharedCurvenetNodes.size(),
        1
    );

    const SharedCurvenetNode& sharedNode =
        result.sharedCurvenetNodes.front();

    const CutChain& verticalChain =
        result.cutChainsByCurveId.at(0);

    const CutChain& branchChain =
        result.cutChainsByCurveId.at(1);

    /*
        The shared Curvenet node should map to the
        same mesh vertex used by both CutChains.
    */
    EXPECT_EQ(
        sharedNode.meshVertexId,
        verticalChain.vertexIds[1]
    );

    EXPECT_EQ(
        sharedNode.meshVertexId,
        branchChain.vertexIds.front()
    );

    /*
        The shared node should record both
        connected profile-curve IDs.
    */
    ASSERT_EQ(
        sharedNode.connectedCurveIds.size(),
        2
    );

    EXPECT_NE(
        std::find(
            sharedNode.connectedCurveIds.begin(),
            sharedNode.connectedCurveIds.end(),
            0
        ),
        sharedNode.connectedCurveIds.end()
    );

    EXPECT_NE(
        std::find(
            sharedNode.connectedCurveIds.begin(),
            sharedNode.connectedCurveIds.end(),
            1
        ),
        sharedNode.connectedCurveIds.end()
    );
}
