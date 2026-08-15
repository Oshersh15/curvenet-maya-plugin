/* Tests arc-length sampling, adaptive counts, and invalid inputs. */

#include "ProfileCurveSampler.h"

#include <gtest/gtest.h>

TEST(ProfileCurveSampler, SamplesStraightLineEvenly)
{
    std::vector<Point3> densePoints;
    densePoints.push_back(Point3{0.0, 0.0, 0.0});
    densePoints.push_back(Point3{5.0, 0.0, 0.0});
    densePoints.push_back(Point3{10.0, 0.0, 0.0});

    std::vector<Point3> sampledPoints =
        ProfileCurveSampler::sampleByArcLength(densePoints, 6);

    ASSERT_EQ(sampledPoints.size(), 6);

    EXPECT_DOUBLE_EQ(sampledPoints[0].x, 0.0);
    EXPECT_DOUBLE_EQ(sampledPoints[1].x, 2.0);
    EXPECT_DOUBLE_EQ(sampledPoints[2].x, 4.0);
    EXPECT_DOUBLE_EQ(sampledPoints[3].x, 6.0);
    EXPECT_DOUBLE_EQ(sampledPoints[4].x, 8.0);
    EXPECT_DOUBLE_EQ(sampledPoints[5].x, 10.0);
}

TEST(ProfileCurveSampler, KeepsFirstAndLastPoint)
{
    std::vector<Point3> densePoints;
    densePoints.push_back(Point3{1.0, 2.0, 3.0});
    densePoints.push_back(Point3{4.0, 2.0, 3.0});
    densePoints.push_back(Point3{7.0, 2.0, 3.0});

    std::vector<Point3> sampledPoints =
        ProfileCurveSampler::sampleByArcLength(densePoints, 4);

    ASSERT_EQ(sampledPoints.size(), 4);

    EXPECT_DOUBLE_EQ(sampledPoints.front().x, 1.0);
    EXPECT_DOUBLE_EQ(sampledPoints.front().y, 2.0);
    EXPECT_DOUBLE_EQ(sampledPoints.front().z, 3.0);

    EXPECT_DOUBLE_EQ(sampledPoints.back().x, 7.0);
    EXPECT_DOUBLE_EQ(sampledPoints.back().y, 2.0);
    EXPECT_DOUBLE_EQ(sampledPoints.back().z, 3.0);
}

TEST(ProfileCurveSampler, ReturnsEmptyForTooFewInputPoints)
{
    std::vector<Point3> densePoints;
    densePoints.push_back(Point3{0.0, 0.0, 0.0});

    std::vector<Point3> sampledPoints =
        ProfileCurveSampler::sampleByArcLength(densePoints, 4);

    EXPECT_TRUE(sampledPoints.empty());
}

TEST(ProfileCurveSampler, ReturnsEmptyForInvalidSampleCount)
{
    std::vector<Point3> densePoints;
    densePoints.push_back(Point3{0.0, 0.0, 0.0});
    densePoints.push_back(Point3{10.0, 0.0, 0.0});

    std::vector<Point3> sampledPoints =
        ProfileCurveSampler::sampleByArcLength(densePoints, 1);

    EXPECT_TRUE(sampledPoints.empty());
}

TEST(ProfileCurveSampler, ReturnsEmptyForZeroLengthCurve)
{
    std::vector<Point3> densePoints;
    densePoints.push_back(Point3{1.0, 1.0, 1.0});
    densePoints.push_back(Point3{1.0, 1.0, 1.0});
    densePoints.push_back(Point3{1.0, 1.0, 1.0});

    std::vector<Point3> sampledPoints =
        ProfileCurveSampler::sampleByArcLength(densePoints, 3);

    EXPECT_TRUE(sampledPoints.empty());
}

TEST(ProfileCurveSampler, ComputesControlPolygonLength)
{
    std::vector<Point3> controlPoints;
    controlPoints.push_back(Point3{0.0, 0.0, 0.0});
    controlPoints.push_back(Point3{3.0, 0.0, 0.0});
    controlPoints.push_back(Point3{3.0, 4.0, 0.0});

    double length =
        ProfileCurveSampler::computeControlPolygonLength(controlPoints);

    EXPECT_DOUBLE_EQ(length, 7.0);
}

TEST(ProfileCurveSampler, ControlPolygonLengthReturnsZeroForTooFewPoints)
{
    std::vector<Point3> controlPoints;
    controlPoints.push_back(Point3{1.0, 2.0, 3.0});

    double length =
        ProfileCurveSampler::computeControlPolygonLength(controlPoints);

    EXPECT_DOUBLE_EQ(length, 0.0);
}

TEST(ProfileCurveSampler, AdaptiveSampleCountMatchesPaperFormula)
{
    int sampleCount =
        ProfileCurveSampler::computeAdaptiveSampleCount(
            10.0,
            2.0
        );

    EXPECT_EQ(sampleCount, 25);
}

TEST(ProfileCurveSampler, AdaptiveSampleCountRoundsUp)
{
    int sampleCount =
        ProfileCurveSampler::computeAdaptiveSampleCount(
            9.0,
            2.0
        );

    EXPECT_EQ(sampleCount, 23);
}

TEST(ProfileCurveSampler, AdaptiveSampleCountReturnsTwoForInvalidInput)
{
    int sampleCount =
        ProfileCurveSampler::computeAdaptiveSampleCount(
            10.0,
            0.0
        );

    EXPECT_EQ(sampleCount, 2);
}

TEST(ProfileCurveSampler, AdaptiveSampleCountNeverReturnsLessThanTwo)
{
    int sampleCount =
        ProfileCurveSampler::computeAdaptiveSampleCount(
            0.01,
            100.0
        );

    EXPECT_EQ(sampleCount, 2);
}

TEST(ProfileCurveSampler, BuildsPolylineSegmentsFromSampledPoints)
{
    std::vector<Point3> sampledPoints;
    sampledPoints.push_back(Point3{0.0, 0.0, 0.0});
    sampledPoints.push_back(Point3{1.0, 0.0, 0.0});
    sampledPoints.push_back(Point3{2.0, 0.0, 0.0});

    std::vector<PolylineSegment> segments =
        ProfileCurveSampler::buildPolylineSegments(sampledPoints);

    ASSERT_EQ(segments.size(), 2);

    EXPECT_DOUBLE_EQ(segments[0].start.x, 0.0);
    EXPECT_DOUBLE_EQ(segments[0].end.x, 1.0);

    EXPECT_DOUBLE_EQ(segments[1].start.x, 1.0);
    EXPECT_DOUBLE_EQ(segments[1].end.x, 2.0);
}

TEST(ProfileCurveSampler, BuildsNoPolylineSegmentsForTooFewPoints)
{
    std::vector<Point3> sampledPoints;
    sampledPoints.push_back(Point3{0.0, 0.0, 0.0});

    std::vector<PolylineSegment> segments =
        ProfileCurveSampler::buildPolylineSegments(sampledPoints);

    EXPECT_TRUE(segments.empty());
}
