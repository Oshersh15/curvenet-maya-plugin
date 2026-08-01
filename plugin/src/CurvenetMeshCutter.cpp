#include "CurvenetMeshCutter.h"

#include "CutPathMeshSplitter.h"

CurvenetCutResult CurvenetMeshCutter::apply(
    const HalfEdgeMesh& inputMesh,
    const std::vector<CutPath>& cutPaths,
    double duplicateTolerance
)
{
    CurvenetCutResult result;

    /*
        Work on a copy so the caller's input mesh
        remains unchanged.
    */
    result.mesh = inputMesh;

    for (const CutPath& cutPath : cutPaths)
    {
        CutPathSplitResult profileResult =
            CutPathMeshSplitter::apply(
                result.mesh,
                cutPath,
                duplicateTolerance
            );

        if (!profileResult.success)
        {
            return result;
        }

        if (
            result.cutChainsByCurveId.find(
                cutPath.curveId
            ) != result.cutChainsByCurveId.end()
        )
        {
            return result;
        }

        result.profileResults.push_back(
            profileResult
        );

        result.cutChainsByCurveId[
            cutPath.curveId
        ] = profileResult.cutChain;
    }

    result.success = true;

    return result;
}
