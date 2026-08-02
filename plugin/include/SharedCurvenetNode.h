#pragma once

#include <vector>

struct SharedCurvenetNode
{
    /*
        Mesh vertex representing this
        shared Curvenet node.
    */
    int meshVertexId = -1;

    /*
        All profile curves connected
        through this shared node.
    */
    std::vector<int> connectedCurveIds;
};
