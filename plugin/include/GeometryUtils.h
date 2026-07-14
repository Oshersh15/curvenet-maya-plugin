#pragma once

#include "HalfEdge.h"
#include "ProfileCurveSampler.h"

/*
    Stores the result of projecting a point onto a segment.

    point:
    - the closest point on the segment

    t:
    - the position along the segment
    - 0 means the segment start
    - 1 means the segment end
*/
struct ClosestPointResult
{
    Point3 point;
    double t = 0.0;
};

struct ClosestCurveSegmentResult
{
    bool found = false;
    int segmentId = -1;
    double segmentT = 0.0;
    Point3 closestPoint;
    double distance = 0.0;
};

/*
    Stores the result of the shortest distance query between two
    finite line segments.
*/
struct SegmentDistanceResult
{
    Point3 closestPointOnFirstSegment;
    Point3 closestPointOnSecondSegment;

    double firstSegmentT = 0.0;
    double secondSegmentT = 0.0;

    double distance = 0.0;
};

/*
    Utility functions for simple 3D geometry operations.

    These functions are independent from the Maya API so they can be
    tested with GoogleTest and reused by curve/mesh intersection code.
*/
namespace GeometryUtils
{
    /*
        Returns the vector from b to a.

        Example:
        a = (5, 7, 2)
        b = (1, 4, 2)

        subtract(a, b) = (4, 3, 0)
    */
    Point3 subtract(
        const Point3& a,
        const Point3& b
    );

    /*
        Computes the dot product between two 3D vectors.

        The dot product measures how much two vectors point in the same
        direction. It is used by closest-point and intersection algorithms.
    */
    double dot(
        const Point3& a,
        const Point3& b
    );

    /*
        Returns the Euclidean length (magnitude) of a 3D vector.
    */
    double length(
        const Point3& vector
    );

    /*
        Returns the Euclidean distance between two 3D points.
    */
    double pointToPointDistance(
        const Point3& a,
        const Point3& b
    );

    /*
        Restricts a value to the range [minimum, maximum].

        If the value is smaller than the minimum, the minimum is returned.
        If the value is larger than the maximum, the maximum is returned.
    */
    double clamp(
        double value,
        double minimum,
        double maximum
    );

    /*
        Adds a scaled direction vector to a start point.

        This is useful for evaluating a point along a segment:

        result = start + direction * scale
    */
    Point3 addScaled(
        const Point3& start,
        const Point3& direction,
        double scale
    );

    /*
        Finds the closest point on a finite segment to a given point.

        The returned t value describes where the closest point lies along
        the segment:
        - t = 0 at segmentStart
        - t = 1 at segmentEnd
    */
    ClosestPointResult closestPointOnSegment(
        const Point3& point,
        const Point3& segmentStart,
        const Point3& segmentEnd
    );

    /*
        Computes the shortest distance between two finite 3D segments.
    */
    SegmentDistanceResult segmentToSegmentDistance(
        const Point3& firstSegmentStart,
        const Point3& firstSegmentEnd,
        const Point3& secondSegmentStart,
        const Point3& secondSegmentEnd
    );

    /*
        Finds the sampled polyline segment closest to a 3D point.
    */
    ClosestCurveSegmentResult findClosestPolylineSegment(
        const Point3& point,
        const std::vector<PolylineSegment>& segments
    );
}
