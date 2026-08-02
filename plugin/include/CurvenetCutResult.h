#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "CutPathMeshSplitter.h"
#include "SharedCurvenetNode.h"

struct CurvenetCutResult
{
    bool success = false;

    HalfEdgeMesh mesh;
    std::unordered_set<int> embeddedVertexIds;
    std::unordered_set<int> embeddedHalfEdgeIds;
    std::unordered_set<int> embeddedFaceIds;

    std::vector<CutPathSplitResult>
        profileResults;

    std::unordered_map<int, CutChain>
        cutChainsByCurveId;

    std::vector<SharedCurvenetNode>
        sharedCurvenetNodes;
};
