#pragma once

#include "HalfEdge.h"
#include <vector>

struct PolylineSegment
{
    Point3 start;
    Point3 end;
};

/*
    Converts a dense profile curve polyline into evenly spaced sample points.

    This class is independent from the Maya API so the sampling logic can be
    tested with GoogleTest.

    The input curve is expected to already be represented as a polyline:
    a list of Point3 values connected in order.
*/
class ProfileCurveSampler
{
public:
    static double computeControlPolygonLength(
        const std::vector<Point3>& controlPoints
    );

    static int computeAdaptiveSampleCount(
        double controlPolygonLength,
        double meanMeshEdgeLength,
        int densityMultiplier = 5 /* later can be exposed as a Maya attribute or UI control */
    );

    static std::vector<PolylineSegment> buildPolylineSegments(
        const std::vector<Point3>& sampledPoints
    );

    /*
        Samples a polyline using approximate arc length.

        Parameters:
        - points: ordered dense polyline points
        - sampleCount: number of output points requested

        Returns:
        - evenly spaced points from the start to the end of the input polyline
        - an empty vector if the input is invalid
    */
    static std::vector<Point3> sampleByArcLength(
        const std::vector<Point3>& points,
        int sampleCount
    );
};
