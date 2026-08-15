#pragma once

/* Identifies a mesh vertex and its corresponding profile-curve parameter. */

#include "HalfEdge.h"

struct EmbeddedCurvePoint
{
    /*
        Embedded mesh vertex.
    */
    int meshVertexId = -1;

    /*
        Location on the sampled profile curve.
    */
    int curveSegmentId = -1;

    double curveSegmentT = 0.0;

    /*
        World-space position.
    */
    Point3 position;
};
