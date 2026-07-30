#pragma once

#include <vector>

struct CutChain
{
    int curveId = -1;

    std::vector<int> vertexIds;
    std::vector<int> halfEdgeIds;

    bool closed = false;
};
