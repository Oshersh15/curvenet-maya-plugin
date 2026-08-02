#include <gtest/gtest.h>

#include <set>

#include "CurvenetFaceBuilder.h"

TEST(
    CurvenetFaceBuilder,
    BuildsOrderedFourCurveFace
)
{
    CurvenetCutResult cutResult;

    /*
        Logical Curvenet loop:

        0 -------- 1
        |          |
        |          |
        3 -------- 2

        Curve 0: 0 -> 1
        Curve 1: 1 -> 2
        Curve 2: 2 -> 3
        Curve 3: 3 -> 0
    */
    cutResult.mesh.vertices.resize(4);

    cutResult.mesh.vertices[0].position =
        Point3{0.0, 1.0, 0.0};

    cutResult.mesh.vertices[1].position =
        Point3{1.0, 1.0, 0.0};

    cutResult.mesh.vertices[2].position =
        Point3{1.0, 0.0, 0.0};

    cutResult.mesh.vertices[3].position =
        Point3{0.0, 0.0, 0.0};

    CutChain topChain;
    topChain.curveId = 0;
    topChain.closed = false;
    topChain.vertexIds = {
        0,
        1
    };

    CutChain rightChain;
    rightChain.curveId = 1;
    rightChain.closed = false;
    rightChain.vertexIds = {
        1,
        2
    };

    CutChain bottomChain;
    bottomChain.curveId = 2;
    bottomChain.closed = false;
    bottomChain.vertexIds = {
        2,
        3
    };

    CutChain leftChain;
    leftChain.curveId = 3;
    leftChain.closed = false;
    leftChain.vertexIds = {
        3,
        0
    };

    cutResult.cutChainsByCurveId[0] =
        topChain;

    cutResult.cutChainsByCurveId[1] =
        rightChain;

    cutResult.cutChainsByCurveId[2] =
        bottomChain;

    cutResult.cutChainsByCurveId[3] =
        leftChain;

    /*
        Every corner is shared by two curves.
    */
    SharedCurvenetNode node0;
    node0.meshVertexId = 0;
    node0.connectedCurveIds = {
        0,
        3
    };

    SharedCurvenetNode node1;
    node1.meshVertexId = 1;
    node1.connectedCurveIds = {
        0,
        1
    };

    SharedCurvenetNode node2;
    node2.meshVertexId = 2;
    node2.connectedCurveIds = {
        1,
        2
    };

    SharedCurvenetNode node3;
    node3.meshVertexId = 3;
    node3.connectedCurveIds = {
        2,
        3
    };

    cutResult.sharedCurvenetNodes = {
        node0,
        node1,
        node2,
        node3
    };

    const std::vector<CurvenetFace> faces =
        CurvenetFaceBuilder::build(
            cutResult
        );

    ASSERT_EQ(
        faces.size(),
        1
    );

    const CurvenetFace& face =
        faces.front();

    EXPECT_EQ(
        face.id,
        0
    );

    ASSERT_EQ(
        face.boundary.size(),
        4
    );

    /*
        All four profile curves should appear
        exactly once in the face boundary.
    */
    std::set<int> boundaryCurveIds;

    for (const CurvenetFaceBoundary& boundary :
         face.boundary)
    {
        boundaryCurveIds.insert(
            boundary.curveId
        );
    }

    EXPECT_EQ(
        boundaryCurveIds,
        std::set<int>({
            0,
            1,
            2,
            3
        })
    );

    /*
        The boundary sections must form one
        continuous and closed traversal.
    */
    for (int boundaryIndex = 0;
         boundaryIndex <
             static_cast<int>(
                 face.boundary.size()
             );
         ++boundaryIndex)
    {
        const CurvenetFaceBoundary& currentBoundary =
            face.boundary[
                boundaryIndex
            ];

        const CurvenetFaceBoundary& nextBoundary =
            face.boundary[
                (boundaryIndex + 1) %
                face.boundary.size()
            ];

        EXPECT_EQ(
            currentBoundary.endVertexId,
            nextBoundary.startVertexId
        );
    }

    /*
        The stored reversed flag must agree with
        the original CutChain direction.
    */
    for (const CurvenetFaceBoundary& boundary :
         face.boundary)
    {
        const CutChain& chain =
            cutResult.cutChainsByCurveId.at(
                boundary.curveId
            );

        ASSERT_FALSE(
            chain.vertexIds.empty()
        );

        if (boundary.reversed)
        {
            EXPECT_EQ(
                boundary.startVertexId,
                chain.vertexIds.back()
            );

            EXPECT_EQ(
                boundary.endVertexId,
                chain.vertexIds.front()
            );
        }
        else
        {
            EXPECT_EQ(
                boundary.startVertexId,
                chain.vertexIds.front()
            );

            EXPECT_EQ(
                boundary.endVertexId,
                chain.vertexIds.back()
            );
        }
    }
}

TEST(
    CurvenetFaceBuilder,
    BuildsOrderedFiveCurveFace
)
{
    CurvenetCutResult cutResult;

    /*
        Five-sided logical Curvenet loop:

            0 -------- 1
            |          |
            4          2
             \        /
                3

        Curve 0: 0 -> 1
        Curve 1: 1 -> 2
        Curve 2: 2 -> 3
        Curve 3: 3 -> 4
        Curve 4: 4 -> 0
    */
    cutResult.mesh.vertices.resize(5);

    cutResult.mesh.vertices[0].position =
        Point3{0.0, 1.0, 0.0};

    cutResult.mesh.vertices[1].position =
        Point3{1.0, 1.0, 0.0};

    cutResult.mesh.vertices[2].position =
        Point3{1.0, 0.4, 0.0};

    cutResult.mesh.vertices[3].position =
        Point3{0.5, 0.0, 0.0};

    cutResult.mesh.vertices[4].position =
        Point3{0.0, 0.4, 0.0};

    CutChain firstChain;
    firstChain.curveId = 0;
    firstChain.closed = false;
    firstChain.vertexIds = {
        0,
        1
    };

    CutChain secondChain;
    secondChain.curveId = 1;
    secondChain.closed = false;
    secondChain.vertexIds = {
        1,
        2
    };

    CutChain thirdChain;
    thirdChain.curveId = 2;
    thirdChain.closed = false;
    thirdChain.vertexIds = {
        2,
        3
    };

    CutChain fourthChain;
    fourthChain.curveId = 3;
    fourthChain.closed = false;
    fourthChain.vertexIds = {
        3,
        4
    };

    CutChain fifthChain;
    fifthChain.curveId = 4;
    fifthChain.closed = false;
    fifthChain.vertexIds = {
        4,
        0
    };

    cutResult.cutChainsByCurveId[0] =
        firstChain;

    cutResult.cutChainsByCurveId[1] =
        secondChain;

    cutResult.cutChainsByCurveId[2] =
        thirdChain;

    cutResult.cutChainsByCurveId[3] =
        fourthChain;

    cutResult.cutChainsByCurveId[4] =
        fifthChain;

    /*
        Every corner is shared by two curves.
    */
    SharedCurvenetNode node0;
    node0.meshVertexId = 0;
    node0.connectedCurveIds = {
        0,
        4
    };

    SharedCurvenetNode node1;
    node1.meshVertexId = 1;
    node1.connectedCurveIds = {
        0,
        1
    };

    SharedCurvenetNode node2;
    node2.meshVertexId = 2;
    node2.connectedCurveIds = {
        1,
        2
    };

    SharedCurvenetNode node3;
    node3.meshVertexId = 3;
    node3.connectedCurveIds = {
        2,
        3
    };

    SharedCurvenetNode node4;
    node4.meshVertexId = 4;
    node4.connectedCurveIds = {
        3,
        4
    };

    cutResult.sharedCurvenetNodes = {
        node0,
        node1,
        node2,
        node3,
        node4
    };

    const std::vector<CurvenetFace> faces =
        CurvenetFaceBuilder::build(
            cutResult
        );

    ASSERT_EQ(
        faces.size(),
        1
    );

    const CurvenetFace& face =
        faces.front();

    EXPECT_EQ(
        face.id,
        0
    );

    ASSERT_EQ(
        face.boundary.size(),
        5
    );

    /*
        All five profile curves should appear
        exactly once in the face boundary.
    */
    std::set<int> boundaryCurveIds;

    for (const CurvenetFaceBoundary& boundary :
         face.boundary)
    {
        boundaryCurveIds.insert(
            boundary.curveId
        );
    }

    EXPECT_EQ(
        boundaryCurveIds,
        std::set<int>({
            0,
            1,
            2,
            3,
            4
        })
    );

    /*
        The boundary sections must form one
        continuous and closed traversal.
    */
    for (int boundaryIndex = 0;
         boundaryIndex <
             static_cast<int>(
                 face.boundary.size()
             );
         ++boundaryIndex)
    {
        const CurvenetFaceBoundary& currentBoundary =
            face.boundary[
                boundaryIndex
            ];

        const CurvenetFaceBoundary& nextBoundary =
            face.boundary[
                (boundaryIndex + 1) %
                face.boundary.size()
            ];

        EXPECT_EQ(
            currentBoundary.endVertexId,
            nextBoundary.startVertexId
        );
    }

    /*
        The stored reversed flag must agree with
        the original CutChain direction.
    */
    for (const CurvenetFaceBoundary& boundary :
         face.boundary)
    {
        const CutChain& chain =
            cutResult.cutChainsByCurveId.at(
                boundary.curveId
            );

        ASSERT_FALSE(
            chain.vertexIds.empty()
        );

        if (boundary.reversed)
        {
            EXPECT_EQ(
                boundary.startVertexId,
                chain.vertexIds.back()
            );

            EXPECT_EQ(
                boundary.endVertexId,
                chain.vertexIds.front()
            );
        }
        else
        {
            EXPECT_EQ(
                boundary.startVertexId,
                chain.vertexIds.front()
            );

            EXPECT_EQ(
                boundary.endVertexId,
                chain.vertexIds.back()
            );
        }
    }
}
