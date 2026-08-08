#pragma once

#include <vector>

#include "EmbeddedCurvePoint.h"

struct CutChain
{
    int curveId = -1;

    std::vector<int> vertexIds;

    /*
        Original CutVertex index corresponding
        to each embedded mesh vertex.
    */
    std::vector<int> cutVertexIndices;

    std::vector<int> halfEdgeIds;

    bool closed = false;

    /*
        Shared Curvenet node at the
        beginning of this CutChain.
    */
    int startSharedNodeId = -1;

    /*
        Shared Curvenet node at the
        end of this CutChain.
    */
    int endSharedNodeId = -1;

    std::vector<EmbeddedCurvePoint> points;
};
