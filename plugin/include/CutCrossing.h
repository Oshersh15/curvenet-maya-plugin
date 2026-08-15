#pragma once

/* Describes one ordered intersection between a profile and a mesh edge. */

#include "HalfEdge.h"

/*
    Stores one detected crossing between a sampled profile curve
    and an edge of the mesh.

    The IDs identify the curve, sampled curve segment, mesh face,
    and half-edge associated with the crossing.

    The position is stored on the mesh edge because the eventual
    cut belongs to the mesh surface.
*/
struct CutCrossing
{
    int curveId = -1;
    int curveSegmentId = -1;
    double curveSegmentT = 0.0;
    int faceId = -1;
    int halfEdgeId = -1;
    double meshEdgeT = 0.0;

    Point3 position;
};
