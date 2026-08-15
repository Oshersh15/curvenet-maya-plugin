#pragma once

/* Stores a logical Curvenet edge and its embedded mesh path. */

#include "HalfEdge.h"

#include <vector>

struct CurvenetEdge
{
    /*
        Unique identifier.
    */
    int id = -1;

    /*
        Shared Curvenet node where
        this edge begins.
    */
    int startNode = -1;

    /*
        Shared Curvenet node where
        this edge ends.
    */
    int endNode = -1;

    /*
        Profile curve from which
        this edge originated.
    */
    int sourceCurveId = -1;

    /*
        Polyline representing this
        Curvenet edge.
    */
    std::vector<Point3> sampledPoints;
};
