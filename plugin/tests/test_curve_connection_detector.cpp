/* Tests endpoint connection detection, including tolerance and duplicates. */

#include "CurveConnectionDetector.h"

#include <gtest/gtest.h>

TEST(
    CurveConnectionDetector,
    DetectsEndpointToCurveConnection
)
{
    const std::vector<std::vector<Point3>> curves
    {
        {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0}
        },
        {
            Point3{1.0, -1.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{1.0, 1.0, 0.0}
        }
    };

    const std::vector<DetectedCurveConnection>
        connections =
            CurveConnectionDetector::detect(
                curves,
                0.0001
            );

    ASSERT_EQ(connections.size(), 1);

    EXPECT_EQ(
        connections[0].endpointCurveId,
        0
    );

    EXPECT_EQ(
        connections[0].endpoint,
        CurveEndpoint::End
    );

    EXPECT_EQ(
        connections[0].targetCurveId,
        1
    );

    EXPECT_EQ(
        connections[0].targetSegmentId,
        0
    );

    EXPECT_DOUBLE_EQ(
        connections[0].targetSegmentT,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        connections[0].position.x,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        connections[0].position.y,
        0.0
    );

    EXPECT_DOUBLE_EQ(
        connections[0].position.z,
        0.0
    );
}

TEST(
    CurveConnectionDetector,
    RejectsConnectionOutsideTolerance
)
{
    const std::vector<std::vector<Point3>> curves
    {
        {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0}
        },
        {
            Point3{1.1, -1.0, 0.0},
            Point3{1.1, 1.0, 0.0}
        }
    };

    const std::vector<DetectedCurveConnection>
        connections =
            CurveConnectionDetector::detect(
                curves,
                0.01
            );

    EXPECT_TRUE(connections.empty());
}

TEST(
    CurveConnectionDetector,
    RemovesReverseDuplicateConnection
)
{
    const std::vector<std::vector<Point3>> curves
    {
        {
            Point3{-1.0, 0.0, 0.0},
            Point3{0.0, 0.0, 0.0}
        },
        {
            Point3{0.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0}
        }
    };

    const std::vector<DetectedCurveConnection>
        connections =
            CurveConnectionDetector::detect(
                curves,
                0.0001
            );

    ASSERT_EQ(connections.size(), 1);

    EXPECT_DOUBLE_EQ(
        connections[0].position.x,
        0.0
    );

    EXPECT_DOUBLE_EQ(
        connections[0].position.y,
        0.0
    );

    EXPECT_DOUBLE_EQ(
        connections[0].position.z,
        0.0
    );
}
