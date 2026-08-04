#pragma once

#include <vector>

#include "CutPath.h"
#include "CurvenetCutResult.h"
#include "HalfEdge.h"
#include "ProfileCutInput.h"

class CurvenetMeshCutter
{
public:

    /*
            Existing overload for already-built CutPaths.
            Kept temporarily for existing tests.
    */
    static CurvenetCutResult apply(
        const HalfEdgeMesh& inputMesh,
        const std::vector<CutPath>& cutPaths,
        double duplicateTolerance
    );

    /*
        Builds each CutPath against the current
        evolving mesh before applying it.
    */
    static CurvenetCutResult apply(
        const HalfEdgeMesh& inputMesh,
        const std::vector<ProfileCutInput>& profileInputs,
        double crossingTolerance,
        double duplicateTolerance
    );
};
