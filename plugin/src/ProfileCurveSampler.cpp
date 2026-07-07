#include "ProfileCurveSampler.h"

#include <cmath>
#include <algorithm>

namespace
{
    double distanceBetween(const Point3& first, const Point3& second)
    {
        const double dx = second.x - first.x;
        const double dy = second.y - first.y;
        const double dz = second.z - first.z;

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Point3 interpolate(const Point3& start, const Point3& end, double t)
    {
        Point3 result;

        result.x = start.x + (end.x - start.x) * t;
        result.y = start.y + (end.y - start.y) * t;
        result.z = start.z + (end.z - start.z) * t;

        return result;
    }
}

double ProfileCurveSampler::computeControlPolygonLength(
    const std::vector<Point3>& controlPoints
)
{
    if (controlPoints.size() < 2)
    {
        return 0.0;
    }

    double totalLength = 0.0;

    for (size_t pointIndex = 0;
         pointIndex < controlPoints.size() - 1;
         ++pointIndex)
    {
        const Point3& startPoint = controlPoints[pointIndex];
        const Point3& endPoint = controlPoints[pointIndex + 1];

        const double dx = endPoint.x - startPoint.x;
        const double dy = endPoint.y - startPoint.y;
        const double dz = endPoint.z - startPoint.z;

        totalLength += std::sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        );
    }

    return totalLength;
}

int ProfileCurveSampler::computeAdaptiveSampleCount(
    double controlPolygonLength,
    double meanMeshEdgeLength,
    int densityMultiplier
)
{
    if (controlPolygonLength <= 0.0 ||
        meanMeshEdgeLength <= 0.0 ||
        densityMultiplier <= 0)
    {
        return 2;
    }

    const double computedSampleCount =
        static_cast<double>(densityMultiplier) *
        (controlPolygonLength / meanMeshEdgeLength);

    return std::max(
        2,
        static_cast<int>(std::ceil(computedSampleCount))
    );
}

std::vector<PolylineSegment> ProfileCurveSampler::buildPolylineSegments(
    const std::vector<Point3>& sampledPoints
)
{
    std::vector<PolylineSegment> segments;

    if (sampledPoints.size() < 2)
    {
        return segments;
    }

    for (size_t pointIndex = 0;
         pointIndex < sampledPoints.size() - 1;
         ++pointIndex)
    {
        PolylineSegment segment;
        segment.start = sampledPoints[pointIndex];
        segment.end = sampledPoints[pointIndex + 1];

        segments.push_back(segment);
    }

    return segments;
}

std::vector<Point3> ProfileCurveSampler::sampleByArcLength(
    const std::vector<Point3>& points,
    int sampleCount
)
{
    std::vector<Point3> sampledPoints;

    if (points.size() < 2)
    {
        return sampledPoints;
    }

    if (sampleCount < 2)
    {
        return sampledPoints;
    }

    double totalLength = 0.0;

    for (int i = 0; i < static_cast<int>(points.size()) - 1; ++i)
    {
        totalLength += distanceBetween(points[i], points[i + 1]);
    }

    if (totalLength <= 0.0)
    {
        return sampledPoints;
    }

    const double spacing = totalLength / static_cast<double>(sampleCount - 1);

    sampledPoints.push_back(points.front());

    double travelledDistance = 0.0;
    int currentSegmentIndex = 0;

    for (int sampleIndex = 1; sampleIndex < sampleCount - 1; ++sampleIndex)
    {
        const double targetDistance = spacing * static_cast<double>(sampleIndex);

        while (currentSegmentIndex < static_cast<int>(points.size()) - 1)
        {
            const Point3& segmentStart = points[currentSegmentIndex];
            const Point3& segmentEnd = points[currentSegmentIndex + 1];
            const double segmentLength = distanceBetween(segmentStart, segmentEnd);

            if (travelledDistance + segmentLength >= targetDistance)
            {
                const double distanceInsideSegment = targetDistance - travelledDistance;
                const double t = distanceInsideSegment / segmentLength;

                sampledPoints.push_back(interpolate(segmentStart, segmentEnd, t));
                break;
            }

            travelledDistance += segmentLength;
            ++currentSegmentIndex;
        }
    }

    sampledPoints.push_back(points.back());

    return sampledPoints;
}
