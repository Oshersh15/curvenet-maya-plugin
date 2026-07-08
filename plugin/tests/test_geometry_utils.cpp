#include "GeometryUtils.h"

#include <gtest/gtest.h>

TEST(GeometryUtils, SubtractReturnsVectorFromSecondPointToFirstPoint)
{
    Point3 a{5.0, 7.0, 2.0};
    Point3 b{1.0, 4.0, 2.0};

    Point3 result = GeometryUtils::subtract(a, b);

    EXPECT_DOUBLE_EQ(result.x, 4.0);
    EXPECT_DOUBLE_EQ(result.y, 3.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(GeometryUtils, DotComputesDotProduct)
{
    Point3 a{1.0, 2.0, 3.0};
    Point3 b{4.0, 5.0, 6.0};

    double result = GeometryUtils::dot(a, b);

    EXPECT_DOUBLE_EQ(result, 32.0);
}

TEST(GeometryUtils, LengthComputesVectorMagnitude)
{
    Point3 vector{3.0, 4.0, 0.0};

    double result = GeometryUtils::length(vector);

    EXPECT_DOUBLE_EQ(result, 5.0);
}

TEST(GeometryUtils, LengthOfZeroVectorIsZero)
{
    Point3 vector{0.0, 0.0, 0.0};

    double result = GeometryUtils::length(vector);

    EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(GeometryUtils, PointToPointDistanceComputesDistance)
{
    Point3 a{0.0, 0.0, 0.0};
    Point3 b{3.0, 4.0, 0.0};

    double result =
        GeometryUtils::pointToPointDistance(a, b);

    EXPECT_DOUBLE_EQ(result, 5.0);
}

TEST(GeometryUtils, PointToPointDistanceIsZeroForSamePoint)
{
    Point3 a{1.0, 2.0, 3.0};
    Point3 b{1.0, 2.0, 3.0};

    double result =
        GeometryUtils::pointToPointDistance(a, b);

    EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(GeometryUtils, ClampReturnsValueInsideRange)
{
    double result = GeometryUtils::clamp(0.5, 0.0, 1.0);

    EXPECT_DOUBLE_EQ(result, 0.5);
}

TEST(GeometryUtils, ClampReturnsMinimumWhenValueIsTooSmall)
{
    double result = GeometryUtils::clamp(-0.2, 0.0, 1.0);

    EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(GeometryUtils, ClampReturnsMaximumWhenValueIsTooLarge)
{
    double result = GeometryUtils::clamp(1.4, 0.0, 1.0);

    EXPECT_DOUBLE_EQ(result, 1.0);
}
