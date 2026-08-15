#pragma once

/* Stores a mesh vertex's neutral attachment to a sampled profile segment. */

#include "HalfEdge.h"

struct VertexCurveBinding
{
    int vertexId = -1;
    int curveId = -1;
    int segmentId = -1;

    double segmentT = 0.0;

    Point3 neutralOffset;
};
