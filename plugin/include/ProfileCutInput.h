#pragma once

#include <vector>

#include "ProfileCurveSampler.h"
#include "CurveConnectionDetector.h"

struct ProfileCurveConnection
{
    CurveEndpoint endpoint =
        CurveEndpoint::Start;

    int targetCurveId = -1;

    int targetSegmentId = -1;

    double targetSegmentT = 0.0;
};

struct ProfileCutInput
{
    /*
        Logical profile-curve identifier.
    */
    int curveId = -1;

    int authoredStartNodeId = -1;
    int authoredEndNodeId = -1;

    /*
        Whether the source profile is open
        or closed.
    */
    bool closed = false;

    /*
        Sampled polyline geometry used to
        intersect the current evolving mesh.
    */
    std::vector<PolylineSegment> sampledSegments;

    /*
        Endpoint connections detected before
        mesh embedding.
    */
    std::vector<ProfileCurveConnection> connections;
};
