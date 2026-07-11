#include "CurveMeshIntersector.h"

#include <gtest/gtest.h>

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
