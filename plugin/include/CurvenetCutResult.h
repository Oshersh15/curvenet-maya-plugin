#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "CutPathMeshSplitter.h"
#include "SharedCurvenetNode.h"
#include "CurvenetFace.h"

struct CurvenetCutResult
{
    bool success = false;

    HalfEdgeMesh mesh;
    std::unordered_set<int> embeddedVertexIds;
    std::unordered_set<int> embeddedHalfEdgeIds;
    std::unordered_set<int> embeddedFaceIds;

    std::vector<CutPathSplitResult>
        profileResults;

    std::vector<CutPath> attemptedCutPaths;

    int failedCurveId = -1;

    CutPathSplitFailure failedSplitReason =
        CutPathSplitFailure::None;

    int failedIntervalIndex = -1;
    int failedFirstVertexId = -1;
    int failedSecondVertexId = -1;

    std::unordered_map<int, CutChain>
        cutChainsByCurveId;

    std::vector<SharedCurvenetNode>
        sharedCurvenetNodes;

    std::vector<CurvenetFace>
        curvenetFaces;
};
