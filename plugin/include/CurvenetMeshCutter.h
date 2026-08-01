#pragma once

#include <vector>

#include "CutPath.h"
#include "CurvenetCutResult.h"
#include "HalfEdge.h"

class CurvenetMeshCutter
{
public:
    static CurvenetCutResult apply(
        const HalfEdgeMesh& inputMesh,
        const std::vector<CutPath>& cutPaths,
        double duplicateTolerance
    );
};
