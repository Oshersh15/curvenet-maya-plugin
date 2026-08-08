#pragma once

#include <vector>
#include "HalfEdge.h"

struct SharedCurvenetNode
{
    /*
        Mesh vertex representing this
        shared Curvenet node.
    */
    int meshVertexId = -1;

    /*
        Position of the Curvenet node. For nodes that
        are not actual mesh vertices, this is the
        author-drawn endpoint position.
    */
    Point3 position;

    /*
        All profile curves connected
        through this shared node.
    */
    std::vector<int> connectedCurveIds;
};
