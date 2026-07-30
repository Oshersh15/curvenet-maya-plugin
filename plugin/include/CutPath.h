#pragma once

#include "CutCrossing.h"
#include "CutVertex.h"

#include <vector>

struct CutPath
{
    int curveId = -1;

    std::vector<CutCrossing> crossings;
    std::vector<int> influencedFaceIds;
    std::vector<int> influencedVertexIds;
    std::vector<CutVertex> cutVertices;
    std::vector<int> faceIntervalIds;

    bool closed = false;
};
