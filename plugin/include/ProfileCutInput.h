#pragma once

#include <vector>

#include "ProfileCurveSampler.h"

struct ProfileCutInput
{
    /*
        Logical profile-curve identifier.
    */
    int curveId = -1;

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
};
