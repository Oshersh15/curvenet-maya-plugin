/* Tests logical face tracing for regular, open, and ambiguous graphs. */

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

TEST(
    CurvenetFaceBuilder,
    BuildsTubeBCurvenetFaces
)
{
    CurvenetCutResult cutResult;

    /*
        Unwrapped logical tube topology:

              curve 2
          A ------------ D
          | \            |
          |  \ curve 4   |
     c0   |   \          |   c1
          |    \         |
          B ------------ C
              curve 3

        The closed curves 2 and 3 each provide
        two distinct sections around the cylinder.

        Expected logical faces:

        {0, 1, 2, 3}
        {1, 3, 4}
        {0, 2, 4}
    */

    const int nodeA = 0;
    const int nodeB = 1;
    const int nodeC = 2;
    const int nodeD = 3;

    /*
        Each boundary section receives neighbouring
        vertices so its local direction at a shared
        node can be ordered using the half-edge mesh.
    */
    const int curve0AtB = 4;
    const int curve0AtA = 5;

    const int curve1AtC = 6;
    const int curve1AtD = 7;

    const int curve2FirstAtA = 8;
    const int curve2FirstAtD = 9;
    const int curve2SecondAtD = 10;
    const int curve2SecondAtA = 11;

    const int curve3FirstAtB = 12;
    const int curve3FirstAtC = 13;
    const int curve3SecondAtC = 14;
    const int curve3SecondAtB = 15;

    const int curve4AtB = 16;
    const int curve4AtD = 17;

    cutResult.mesh.vertices.resize(18);

    CutChain curve0;
    curve0.curveId = 0;
    curve0.closed = false;
    curve0.vertexIds = {
        nodeB,
        curve0AtB,
        curve0AtA,
        nodeA
    };

    CutChain curve1;
    curve1.curveId = 1;
    curve1.closed = false;
    curve1.vertexIds = {
        nodeC,
        curve1AtC,
        curve1AtD,
        nodeD
    };

    CutChain curve2;
    curve2.curveId = 2;
    curve2.closed = true;
    curve2.vertexIds = {
        nodeA,
        curve2FirstAtA,
        curve2FirstAtD,
        nodeD,
        curve2SecondAtD,
        curve2SecondAtA
    };

    CutChain curve3;
    curve3.curveId = 3;
    curve3.closed = true;
    curve3.vertexIds = {
        nodeB,
        curve3FirstAtB,
        curve3FirstAtC,
        nodeC,
        curve3SecondAtC,
        curve3SecondAtB
    };

    CutChain curve4;
    curve4.curveId = 4;
    curve4.closed = false;
    curve4.vertexIds = {
        nodeB,
        curve4AtB,
        curve4AtD,
        nodeD
    };

    cutResult.cutChainsByCurveId[0] =
        curve0;

    cutResult.cutChainsByCurveId[1] =
        curve1;

    cutResult.cutChainsByCurveId[2] =
        curve2;

    cutResult.cutChainsByCurveId[3] =
        curve3;

    cutResult.cutChainsByCurveId[4] =
        curve4;

    SharedCurvenetNode sharedNodeA;
    sharedNodeA.meshVertexId = nodeA;
    sharedNodeA.connectedCurveIds = {
        0,
        2
    };

    SharedCurvenetNode sharedNodeB;
    sharedNodeB.meshVertexId = nodeB;
    sharedNodeB.connectedCurveIds = {
        0,
        3,
        4
    };

    SharedCurvenetNode sharedNodeC;
    sharedNodeC.meshVertexId = nodeC;
    sharedNodeC.connectedCurveIds = {
        1,
        3
    };

    SharedCurvenetNode sharedNodeD;
    sharedNodeD.meshVertexId = nodeD;
    sharedNodeD.connectedCurveIds = {
        1,
        2,
        4
    };

    cutResult.sharedCurvenetNodes = {
        sharedNodeA,
        sharedNodeB,
        sharedNodeC,
        sharedNodeD
    };

    /*
        Store outgoing half-edges in their intended
        cyclic order around each shared node.

        Their face IDs remain invalid because this test
        needs only the local ordering of curve directions.
    */
    const auto addOutgoingHalfEdge =
        [&cutResult](
            int startVertexId,
            int endVertexId
        )
        {
            HalfEdge halfEdge;

            halfEdge.startVertex =
                startVertexId;

            halfEdge.endVertex =
                endVertexId;

            halfEdge.face = -1;
            halfEdge.twin = -1;
            halfEdge.next = -1;

            cutResult.mesh.halfEdges.push_back(
                halfEdge
            );
        };

    /*
        Cyclic order at node A.
    */
    addOutgoingHalfEdge(
        nodeA,
        curve2SecondAtA
    );

    addOutgoingHalfEdge(
        nodeA,
        curve0AtA
    );

    addOutgoingHalfEdge(
        nodeA,
        curve2FirstAtA
    );

    /*
        Cyclic order at node B.
    */
    addOutgoingHalfEdge(
        nodeB,
        curve3FirstAtB
    );

    addOutgoingHalfEdge(
        nodeB,
        curve4AtB
    );

    addOutgoingHalfEdge(
        nodeB,
        curve0AtB
    );

    addOutgoingHalfEdge(
        nodeB,
        curve3SecondAtB
    );

    /*
        Cyclic order at node C.
    */
    addOutgoingHalfEdge(
        nodeC,
        curve3SecondAtC
    );

    addOutgoingHalfEdge(
        nodeC,
        curve1AtC
    );

    addOutgoingHalfEdge(
        nodeC,
        curve3FirstAtC
    );

    /*
        Cyclic order at node D.
    */
    addOutgoingHalfEdge(
        nodeD,
        curve2FirstAtD
    );

    addOutgoingHalfEdge(
        nodeD,
        curve4AtD
    );

    addOutgoingHalfEdge(
        nodeD,
        curve1AtD
    );

    addOutgoingHalfEdge(
        nodeD,
        curve2SecondAtD
    );

    const std::vector<CurvenetFace> faces =
        CurvenetFaceBuilder::build(
            cutResult
        );

    ASSERT_EQ(
        faces.size(),
        3
    );

    std::set<std::set<int>>
        actualFaceCurveSets;

    for (const CurvenetFace& face :
         faces)
    {
        ASSERT_GE(
            face.boundary.size(),
            3
        );

        std::set<int> curveIds;

        for (int boundaryIndex = 0;
             boundaryIndex <
                 static_cast<int>(
                     face.boundary.size()
                 );
             ++boundaryIndex)
        {
            const CurvenetFaceBoundary&
                currentBoundary =
                    face.boundary[
                        boundaryIndex
                    ];

            const CurvenetFaceBoundary&
                nextBoundary =
                    face.boundary[
                        (
                            boundaryIndex + 1
                        ) %
                        face.boundary.size()
                    ];

            curveIds.insert(
                currentBoundary.curveId
            );

            /*
                Every candidate must be a continuous,
                closed boundary loop.
            */
            EXPECT_EQ(
                currentBoundary.endVertexId,
                nextBoundary.startVertexId
            );
        }

        actualFaceCurveSets.insert(
            curveIds
        );
    }

    const std::set<std::set<int>>
        expectedFaceCurveSets = {
            {
                0,
                1,
                2,
                3
            },
            {
                1,
                3,
                4
            },
            {
                0,
                2,
                4
            }
        };

    EXPECT_EQ(
        actualFaceCurveSets,
        expectedFaceCurveSets
    );
}
