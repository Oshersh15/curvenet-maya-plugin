#pragma once

#include "HalfEdge.h"

/*
    Stores one vertex that will be inserted into the mesh
    along a CutPath.

    The position is the location of the new vertex on the
    original mesh edge.

    sourceHalfEdgeId identifies the original half-edge that
    contains the cut.

    sourceEdgeT stores where the cut lies along that edge:
        0.0 = at the start vertex
        1.0 = at the end vertex

    curveId identifies the profile curve that produced the cut.

    cutPathOrder stores this vertex's position in the ordered
    sequence of vertices along the CutPath.
*/
struct CutVertex
{
    Point3 position;

    int sourceHalfEdgeId = -1;
    int existingMeshVertexId = -1;
    double sourceEdgeT = 0.0;

    int curveId = -1;
    int cutPathOrder = -1;
};
