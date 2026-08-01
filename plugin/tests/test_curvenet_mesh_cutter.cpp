#include <gtest/gtest.h>

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
