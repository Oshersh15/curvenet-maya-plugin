#pragma once

#include <vector>
#include <unordered_map>

#include "CutPathMeshSplitter.h"

struct CurvenetCutResult
{
    bool success = false;

    HalfEdgeMesh mesh;

    std::vector<CutPathSplitResult>
        profileResults;

    std::unordered_map<int, CutChain>
        cutChainsByCurveId;
};
