#include "GeometryUtils.h"
#include <cmath>

namespace GeometryUtils
{
    Point3 subtract(
        const Point3& a,
        const Point3& b
    )
    {
        return Point3{
            a.x - b.x,
            a.y - b.y,
            a.z - b.z
        };
    }

    double dot(
        const Point3& a,
        const Point3& b
    )
    {
        return
            a.x * b.x +
            a.y * b.y +
            a.z * b.z;
    }

    double length(
        const Point3& vector
    )
    {
        return std::sqrt(
            dot(vector, vector)
        );
    }

    double pointToPointDistance(
        const Point3& a,
        const Point3& b
    )
    {
        Point3 direction =
            subtract(a, b);

        return length(direction);
    }

    double clamp(
        double value,
        double minimum,
        double maximum
    )
    {
        if (value < minimum)
        {
            return minimum;
        }

        if (value > maximum)
        {
            return maximum;
        }

        return value;
    }

    Point3 addScaled(
        const Point3& start,
        const Point3& direction,
        double scale
    )
    {
        return Point3{
            start.x + direction.x * scale,
            start.y + direction.y * scale,
            start.z + direction.z * scale
        };
    }

    ClosestPointResult closestPointOnSegment(
        const Point3& point,
        const Point3& segmentStart,
        const Point3& segmentEnd
    )
    {
        Point3 segmentDirection =
            subtract(segmentEnd, segmentStart);

        Point3 startToPoint =
            subtract(point, segmentStart);

        double segmentLengthSquared =
            dot(segmentDirection, segmentDirection);

        if (segmentLengthSquared <= 0.0)
        {
            return ClosestPointResult{
                segmentStart,
                0.0
            };
        }

        double t =
            dot(startToPoint, segmentDirection) /
            segmentLengthSquared;

        t = clamp(t, 0.0, 1.0);

        Point3 closestPoint =
            addScaled(segmentStart, segmentDirection, t);

        return ClosestPointResult{
            closestPoint,
            t
        };
    }

    ClosestCurveSegmentResult findClosestPolylineSegment(
        const Point3& point,
        const std::vector<PolylineSegment>& segments
    )
    {
        ClosestCurveSegmentResult result;

        if (segments.empty())
        {
            return result;
        }

        for (int segmentId = 0;
             segmentId < static_cast<int>(segments.size());
             ++segmentId)
        {
            const PolylineSegment& segment =
                segments[segmentId];

            const ClosestPointResult closestPointResult =
                closestPointOnSegment(
                    point,
                    segment.start,
                    segment.end
                );

            const double distance =
                pointToPointDistance(
                    point,
                    closestPointResult.point
                );

            if (!result.found ||
                distance < result.distance)
            {
                result.found = true;
                result.segmentId = segmentId;
                result.segmentT = closestPointResult.t;
                result.closestPoint =
                    closestPointResult.point;
                result.distance = distance;
            }
        }

        return result;
    }

    Point3 interpolateSegmentDisplacement(
        const Point3& neutralStart,
        const Point3& neutralEnd,
        const Point3& posedStart,
        const Point3& posedEnd,
        double t
    )
    {
        Point3 startDisplacement =
            subtract(
                posedStart,
                neutralStart
            );

        Point3 endDisplacement =
            subtract(
                posedEnd,
                neutralEnd
            );

        Point3 displacement;

        displacement.x =
            startDisplacement.x +
            (endDisplacement.x - startDisplacement.x) * t;

        displacement.y =
            startDisplacement.y +
            (endDisplacement.y - startDisplacement.y) * t;

        displacement.z =
            startDisplacement.z +
            (endDisplacement.z - startDisplacement.z) * t;

        return displacement;
    }

    SegmentDistanceResult segmentToSegmentDistance(
        const Point3& firstSegmentStart,
        const Point3& firstSegmentEnd,
        const Point3& secondSegmentStart,
        const Point3& secondSegmentEnd
    )
    {
        const Point3 firstDirection =
            subtract(firstSegmentEnd, firstSegmentStart);

        const Point3 secondDirection =
            subtract(secondSegmentEnd, secondSegmentStart);

        const Point3 startDifference =
            subtract(firstSegmentStart, secondSegmentStart);

        const double a = dot(firstDirection, firstDirection);
        const double b = dot(firstDirection, secondDirection);
        const double c = dot(secondDirection, secondDirection);
        const double d = dot(firstDirection, startDifference);
        const double e = dot(secondDirection, startDifference);

        double firstT = 0.0;
        double secondT = 0.0;

        const double epsilon = 1e-8;

        if (a <= epsilon && c <= epsilon)
        {
            firstT = 0.0;
            secondT = 0.0;
        }
        else if (a <= epsilon)
        {
            firstT = 0.0;
            secondT = clamp(e / c, 0.0, 1.0);
        }
        else if (c <= epsilon)
        {
            secondT = 0.0;
            firstT = clamp(-d / a, 0.0, 1.0);
        }
        else
        {
            const double denominator = a * c - b * b;

            if (denominator != 0.0)
            {
                firstT = clamp((b * e - c * d) / denominator, 0.0, 1.0);
            }
            else
            {
                firstT = 0.0;
            }

            secondT = (b * firstT + e) / c;

            if (secondT < 0.0)
            {
                secondT = 0.0;
                firstT = clamp(-d / a, 0.0, 1.0);
            }
            else if (secondT > 1.0)
            {
                secondT = 1.0;
                firstT = clamp((b - d) / a, 0.0, 1.0);
            }
        }

        const Point3 closestPointOnFirstSegment =
            addScaled(firstSegmentStart, firstDirection, firstT);

        const Point3 closestPointOnSecondSegment =
            addScaled(secondSegmentStart, secondDirection, secondT);

        const double distance =
            pointToPointDistance(
                closestPointOnFirstSegment,
                closestPointOnSecondSegment
            );

        return SegmentDistanceResult{
            closestPointOnFirstSegment,
            closestPointOnSecondSegment,
            firstT,
            secondT,
            distance
        };
    }

    bool areSegmentsNearlyParallel(
        const Point3& firstStart,
        const Point3& firstEnd,
        const Point3& secondStart,
        const Point3& secondEnd,
        double parallelTolerance
    )
    {
        const Point3 firstDirection =
            subtract(
                firstEnd,
                firstStart
            );

        const Point3 secondDirection =
            subtract(
                secondEnd,
                secondStart
            );

        const double firstLength =
            length(firstDirection);

        const double secondLength =
            length(secondDirection);

        if (firstLength <= 0.0 ||
            secondLength <= 0.0)
        {
            return false;
        }

        const Point3 firstNormalised{
            firstDirection.x / firstLength,
            firstDirection.y / firstLength,
            firstDirection.z / firstLength
        };

        const Point3 secondNormalised{
            secondDirection.x / secondLength,
            secondDirection.y / secondLength,
            secondDirection.z / secondLength
        };

        const double directionDot =
            dot(
                firstNormalised,
                secondNormalised
            );

        return
            std::abs(directionDot) >=
            1.0 - parallelTolerance;
    }
}
