#pragma once

#include "HalfEdge.h"
#include "ProfileCurveSampler.h"
#include "CutCrossing.h"

/*
    Stores the result of searching for the first crossing between
    a sampled profile-curve polyline and a half-edge mesh.

    A crossing is considered valid when the shortest distance between
    a curve segment and a unique mesh edge is within the supplied
    tolerance.
*/
struct FirstCrossingResult
{
    bool found = false;

    CutCrossing crossing;

    double distance = 0.0;
};

/*
    Provides Maya-independent curve/mesh intersection queries.
*/
class CurveMeshIntersector
{
public:
    /*
        Finds the first valid crossing in curve-segment order.

        Returns immediately when the first curve segment within the
        tolerance of a unique mesh edge is found.
    */
    static FirstCrossingResult findFirstCrossing(
        int curveId,
        const std::vector<PolylineSegment>& curveSegments,
        const HalfEdgeMesh& mesh,
        double tolerance
    );

    /*
        Finds all distinct crossings between a sampled profile curve
        and a half-edge mesh.

        Crossings are returned in curve-segment order.
    */
    static std::vector<CutCrossing> findAllCrossings(
        int curveId,
        const std::vector<PolylineSegment>& curveSegments,
        const HalfEdgeMesh& mesh,
        double crossingTolerance,
        double duplicateTolerance
    );
};
