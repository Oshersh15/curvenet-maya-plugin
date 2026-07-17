#include "CurveMeshIntersector.h"

#include <gtest/gtest.h>
#include "CutPath.h"

TEST(CurveMeshIntersector, FindsFirstCrossingInCurveSegmentOrder)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<PolylineSegment> curveSegments;

    curveSegments.push_back(PolylineSegment{
        Point3{-1.0, 2.0, 0.0},
        Point3{-0.5, 2.0, 0.0}
    });

    curveSegments.push_back(PolylineSegment{
        Point3{-0.5, 0.5, 0.0},
        Point3{0.5, 0.5, 0.0}
    });

    curveSegments.push_back(PolylineSegment{
        Point3{0.5, 0.5, 0.0},
        Point3{1.5, 0.5, 0.0}
    });

    const int curveId = 7;

    FirstCrossingResult result =
        CurveMeshIntersector::findFirstCrossing(
            curveId,
            curveSegments,
            mesh,
            0.0001
        );

    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.crossing.curveId, curveId);
    EXPECT_EQ(result.crossing.curveSegmentId, 1);
    EXPECT_GE(result.crossing.faceId, 0);
    EXPECT_GE(result.crossing.halfEdgeId, 0);
    EXPECT_DOUBLE_EQ(result.distance, 0.0);
}

TEST(CurveMeshIntersector, ReturnsNotFoundWhenNoCrossingExists)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<PolylineSegment> curveSegments;

    curveSegments.push_back(PolylineSegment{
        Point3{-2.0, 3.0, 0.0},
        Point3{-1.0, 3.0, 0.0}
    });

    FirstCrossingResult result =
        CurveMeshIntersector::findFirstCrossing(
            7,
            curveSegments,
            mesh,
            0.0001
        );

    EXPECT_FALSE(result.found);
    EXPECT_EQ(result.crossing.curveId, -1);
    EXPECT_EQ(result.crossing.curveSegmentId, -1);
    EXPECT_EQ(result.crossing.faceId, -1);
    EXPECT_EQ(result.crossing.halfEdgeId, -1);
}

TEST(CurveMeshIntersector, FindsMultipleCrossingsInCurveOrder)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<PolylineSegment> curveSegments;

    curveSegments.push_back(PolylineSegment{
        Point3{-1.0, 0.75, 0.0},
        Point3{0.5, 0.75, 0.0}
    });

    curveSegments.push_back(PolylineSegment{
        Point3{0.5, 0.25, 0.0},
        Point3{2.0, 0.25, 0.0}
    });

    std::vector<CutCrossing> crossings =
        CurveMeshIntersector::findAllCrossings(
            3,
            curveSegments,
            mesh,
            0.0001,
            0.0001
        );

    ASSERT_EQ(crossings.size(), 2);

    EXPECT_EQ(crossings[0].curveId, 3);
    EXPECT_EQ(crossings[0].curveSegmentId, 0);

    EXPECT_EQ(crossings[1].curveId, 3);
    EXPECT_EQ(crossings[1].curveSegmentId, 1);
}

TEST(CurveMeshIntersector, RemovesDuplicateCrossingsAtSamePosition)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<PolylineSegment> curveSegments;

    curveSegments.push_back(PolylineSegment{
        Point3{-1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0}
    });

    std::vector<CutCrossing> crossings =
        CurveMeshIntersector::findAllCrossings(
            5,
            curveSegments,
            mesh,
            0.0001,
            0.0001
        );

    ASSERT_EQ(crossings.size(), 1);

    EXPECT_EQ(crossings[0].curveId, 5);
    EXPECT_DOUBLE_EQ(crossings[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(crossings[0].position.y, 1.0);
    EXPECT_DOUBLE_EQ(crossings[0].position.z, 0.0);
}

TEST(CurveMeshIntersector, ReturnsEmptyWhenNoCrossingsExist)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<PolylineSegment> curveSegments;

    curveSegments.push_back(PolylineSegment{
        Point3{-3.0, 3.0, 0.0},
        Point3{-2.0, 3.0, 0.0}
    });

    std::vector<CutCrossing> crossings =
        CurveMeshIntersector::findAllCrossings(
            2,
            curveSegments,
            mesh,
            0.0001,
            0.0001
        );

    EXPECT_TRUE(crossings.empty());
}

TEST(CutPath, BuildsPathForCurveCrossingSingleFace)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    std::vector<PolylineSegment> curveSegments;

    curveSegments.push_back(PolylineSegment{
        Point3{-1.0, 0.5, 0.0},
        Point3{2.0, 0.5, 0.0}
    });

    std::vector<CutCrossing> crossings =
        CurveMeshIntersector::findAllCrossings(
            0,
            curveSegments,
            mesh,
            0.0001,
            0.0001
        );

    CutPath cutPath;
    cutPath.curveId = 0;
    cutPath.crossings = crossings;

    EXPECT_EQ(cutPath.curveId, 0);
    ASSERT_EQ(cutPath.crossings.size(), 2);

    EXPECT_EQ(cutPath.crossings[0].curveId, 0);
    EXPECT_EQ(cutPath.crossings[1].curveId, 0);

    EXPECT_LE(
        cutPath.crossings[0].curveSegmentId,
        cutPath.crossings[1].curveSegmentId
    );
}

TEST(CutPath, BuildsPathForCurveCrossingMultipleFaces)
{
    HalfEdgeMesh mesh;

    mesh.vertices = {
        Vertex{Point3{0.0, 0.0, 0.0}},
        Vertex{Point3{1.0, 0.0, 0.0}},
        Vertex{Point3{2.0, 0.0, 0.0}},
        Vertex{Point3{0.0, 1.0, 0.0}},
        Vertex{Point3{1.0, 1.0, 0.0}},
        Vertex{Point3{2.0, 1.0, 0.0}}
    };

    mesh.faces = {
        Face{0},
        Face{4}
    };

    mesh.halfEdges = {
        HalfEdge{0, 1, 1, -1, 0},
        HalfEdge{1, 4, 2, 7, 0},
        HalfEdge{4, 3, 3, -1, 0},
        HalfEdge{3, 0, 0, -1, 0},

        HalfEdge{1, 2, 5, -1, 1},
        HalfEdge{2, 5, 6, -1, 1},
        HalfEdge{5, 4, 7, -1, 1},
        HalfEdge{4, 1, 4, 1, 1}
    };

    std::vector<PolylineSegment> curveSegments;

    curveSegments.push_back(PolylineSegment{
        Point3{-0.5, 0.5, 0.0},
        Point3{2.5, 0.5, 0.0}
    });

    std::vector<CutCrossing> crossings =
        CurveMeshIntersector::findAllCrossings(
            0,
            curveSegments,
            mesh,
            0.0001,
            0.0001
        );

    CutPath cutPath;
    cutPath.curveId = 0;
    cutPath.crossings = crossings;

    EXPECT_EQ(cutPath.curveId, 0);
    ASSERT_EQ(cutPath.crossings.size(), 3);

    EXPECT_DOUBLE_EQ(
        cutPath.crossings[0].position.x,
        0.0
    );

    EXPECT_DOUBLE_EQ(
        cutPath.crossings[1].position.x,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        cutPath.crossings[2].position.x,
        2.0
    );
}

TEST(CurveMeshIntersector, DerivesFaceIntervalBetweenTwoCrossings)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutPath cutPath;

    CutCrossing firstCrossing;
    firstCrossing.halfEdgeId = 0;

    CutCrossing secondCrossing;
    secondCrossing.halfEdgeId = 2;

    cutPath.crossings.push_back(firstCrossing);
    cutPath.crossings.push_back(secondCrossing);

    std::vector<int> faceIntervals =
        CurveMeshIntersector::deriveFaceIntervals(
            cutPath,
            mesh
        );

    ASSERT_EQ(faceIntervals.size(), 1);

    EXPECT_EQ(faceIntervals[0], 0);
}

TEST(CurveMeshIntersector, CollectUniqueFacesIgnoresUnresolvedIntervals)
{
    std::vector<int> faceIntervals{
        8,
        -1,
        5,
        -1,
        6
    };

    std::vector<int> collectedFaceIds =
        CurveMeshIntersector::collectUniqueFaces(
            faceIntervals
        );

    ASSERT_EQ(collectedFaceIds.size(), 3);

    EXPECT_EQ(collectedFaceIds[0], 8);
    EXPECT_EQ(collectedFaceIds[1], 5);
    EXPECT_EQ(collectedFaceIds[2], 6);
}

TEST(CurveMeshIntersector, CollectUniqueFacesRemovesDuplicates)
{
    std::vector<int> faceIntervals{
        8,
        8,
        5,
        6,
        5
    };

    std::vector<int> collectedFaceIds =
        CurveMeshIntersector::collectUniqueFaces(
            faceIntervals
        );

    ASSERT_EQ(collectedFaceIds.size(), 3);

    EXPECT_EQ(collectedFaceIds[0], 8);
    EXPECT_EQ(collectedFaceIds[1], 5);
    EXPECT_EQ(collectedFaceIds[2], 6);
}

TEST(CurveMeshIntersector, CollectUniqueFacesPreservesFirstTraversalOrder)
{
    std::vector<int> faceIntervals{
        6,
        8,
        6,
        5,
        8,
        3
    };

    std::vector<int> collectedFaceIds =
        CurveMeshIntersector::collectUniqueFaces(
            faceIntervals
        );

    ASSERT_EQ(collectedFaceIds.size(), 4);

    EXPECT_EQ(collectedFaceIds[0], 6);
    EXPECT_EQ(collectedFaceIds[1], 8);
    EXPECT_EQ(collectedFaceIds[2], 5);
    EXPECT_EQ(collectedFaceIds[3], 3);
}

TEST(
    CurveMeshIntersector,
    BuildsCutVerticesFromOrderedCrossings
)
{
    std::vector<CutCrossing> crossings;

    CutCrossing firstCrossing;
    firstCrossing.curveId = 3;
    firstCrossing.halfEdgeId = 10;
    firstCrossing.meshEdgeT = 0.25;
    firstCrossing.position =
        Point3{1.0, 2.0, 3.0};

    CutCrossing secondCrossing;
    secondCrossing.curveId = 3;
    secondCrossing.halfEdgeId = 20;
    secondCrossing.meshEdgeT = 0.75;
    secondCrossing.position =
        Point3{4.0, 5.0, 6.0};

    crossings.push_back(firstCrossing);
    crossings.push_back(secondCrossing);

    const std::vector<CutVertex> cutVertices =
        CurveMeshIntersector::buildCutVertices(
            crossings,
            0.0001
        );

    ASSERT_EQ(cutVertices.size(), 2);

    EXPECT_EQ(cutVertices[0].curveId, 3);
    EXPECT_EQ(cutVertices[0].sourceHalfEdgeId, 10);
    EXPECT_DOUBLE_EQ(cutVertices[0].sourceEdgeT, 0.25);
    EXPECT_EQ(cutVertices[0].cutPathOrder, 0);

    EXPECT_DOUBLE_EQ(cutVertices[0].position.x, 1.0);
    EXPECT_DOUBLE_EQ(cutVertices[0].position.y, 2.0);
    EXPECT_DOUBLE_EQ(cutVertices[0].position.z, 3.0);

    EXPECT_EQ(cutVertices[1].curveId, 3);
    EXPECT_EQ(cutVertices[1].sourceHalfEdgeId, 20);
    EXPECT_DOUBLE_EQ(cutVertices[1].sourceEdgeT, 0.75);
    EXPECT_EQ(cutVertices[1].cutPathOrder, 1);
}

TEST(
    CurveMeshIntersector,
    ReusesCutVertexForDuplicateCrossingPosition
)
{
    std::vector<CutCrossing> crossings;

    CutCrossing firstCrossing;
    firstCrossing.curveId = 1;
    firstCrossing.halfEdgeId = 8;
    firstCrossing.meshEdgeT = 0.5;
    firstCrossing.position =
        Point3{2.0, 0.0, 1.0};

    CutCrossing duplicateCrossing;
    duplicateCrossing.curveId = 1;
    duplicateCrossing.halfEdgeId = 9;
    duplicateCrossing.meshEdgeT = 0.5;
    duplicateCrossing.position =
        Point3{2.00001, 0.0, 1.0};

    crossings.push_back(firstCrossing);
    crossings.push_back(duplicateCrossing);

    const std::vector<CutVertex> cutVertices =
        CurveMeshIntersector::buildCutVertices(
            crossings,
            0.001
        );

    ASSERT_EQ(cutVertices.size(), 1);

    EXPECT_EQ(cutVertices[0].sourceHalfEdgeId, 8);
    EXPECT_EQ(cutVertices[0].cutPathOrder, 0);
}

TEST(
    CurveMeshIntersector,
    PreservesCutPathOrderAfterRemovingDuplicates
)
{
    std::vector<CutCrossing> crossings;

    CutCrossing firstCrossing;
    firstCrossing.position =
        Point3{0.0, 0.0, 0.0};

    CutCrossing duplicateCrossing;
    duplicateCrossing.position =
        Point3{0.00001, 0.0, 0.0};

    CutCrossing thirdCrossing;
    thirdCrossing.position =
        Point3{2.0, 0.0, 0.0};

    crossings.push_back(firstCrossing);
    crossings.push_back(duplicateCrossing);
    crossings.push_back(thirdCrossing);

    const std::vector<CutVertex> cutVertices =
        CurveMeshIntersector::buildCutVertices(
            crossings,
            0.001
        );

    ASSERT_EQ(cutVertices.size(), 2);

    EXPECT_EQ(cutVertices[0].cutPathOrder, 0);
    EXPECT_EQ(cutVertices[1].cutPathOrder, 1);

    EXPECT_DOUBLE_EQ(
        cutVertices[1].position.x,
        2.0
    );
}
