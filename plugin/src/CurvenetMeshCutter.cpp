#include "CurvenetMeshCutter.h"

#include "CutPathMeshSplitter.h"
#include "CurvenetSharedNodeDetector.h"

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

    for (const CutPath& sourceCutPath : cutPaths)
    {
        CutPath preparedCutPath =
            sourceCutPath;

        int firstEndpointIndex = -1;
        int lastEndpointIndex = -1;

        for (int cutVertexIndex = 0;
             cutVertexIndex <
                 static_cast<int>(
                     preparedCutPath.cutVertices.size()
                 );
             ++cutVertexIndex)
        {
            if (firstEndpointIndex < 0 ||
                preparedCutPath
                    .cutVertices[cutVertexIndex]
                    .cutPathOrder <
                preparedCutPath
                    .cutVertices[firstEndpointIndex]
                    .cutPathOrder)
            {
                firstEndpointIndex =
                    cutVertexIndex;
            }

            if (lastEndpointIndex < 0 ||
                preparedCutPath
                    .cutVertices[cutVertexIndex]
                    .cutPathOrder >
                preparedCutPath
                    .cutVertices[lastEndpointIndex]
                    .cutPathOrder)
            {
                lastEndpointIndex =
                    cutVertexIndex;
            }
        }

        if (firstEndpointIndex >= 0)
        {
            CutVertex& firstEndpoint =
                preparedCutPath
                    .cutVertices[firstEndpointIndex];

            const std::optional<int>
                sharedVertexId =
                    CurvenetSharedNodeDetector::
                        findSharedMeshVertex(
                            firstEndpoint,
                            result,
                            duplicateTolerance
                        );

            if (sharedVertexId.has_value())
            {
                firstEndpoint.existingMeshVertexId =
                    sharedVertexId.value();
            }
        }

        if (lastEndpointIndex >= 0 &&
            lastEndpointIndex != firstEndpointIndex)
        {
            CutVertex& lastEndpoint =
                preparedCutPath
                    .cutVertices[lastEndpointIndex];

            const std::optional<int>
                sharedVertexId =
                    CurvenetSharedNodeDetector::
                        findSharedMeshVertex(
                            lastEndpoint,
                            result,
                            duplicateTolerance
                        );

            if (sharedVertexId.has_value())
            {
                lastEndpoint.existingMeshVertexId =
                    sharedVertexId.value();
            }
        }

        CutPathSplitResult profileResult =
            CutPathMeshSplitter::apply(
                result.mesh,
                preparedCutPath,
                duplicateTolerance
            );

        if (!profileResult.success)
        {
            return result;
        }

        if (
            result.cutChainsByCurveId.find(
                sourceCutPath.curveId
            ) != result.cutChainsByCurveId.end()
        )
        {
            return result;
        }

        result.profileResults.push_back(
            profileResult
        );

        result.cutChainsByCurveId[
            sourceCutPath.curveId
        ] = profileResult.cutChain;
    }

    result.success = true;

    return result;
}
