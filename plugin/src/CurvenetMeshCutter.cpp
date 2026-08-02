#include "CurvenetMeshCutter.h"

#include "CutPathMeshSplitter.h"
#include "CurvenetSharedNodeDetector.h"

#include <algorithm>
#include <unordered_map>

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

        for (int meshVertexId :
             profileResult.cutChain.vertexIds)
        {
            result.embeddedVertexIds.insert(
                meshVertexId
            );
        }

        for (int halfEdgeId :
             profileResult.cutChain.halfEdgeIds)
        {
            result.embeddedHalfEdgeIds.insert(
                halfEdgeId
            );
        }
    }

    for (int halfEdgeId :
         result.embeddedHalfEdgeIds)
    {
        if (halfEdgeId < 0 ||
            halfEdgeId >=
                static_cast<int>(
                    result.mesh.halfEdges.size()
                ))
        {
            return result;
        }

        const HalfEdge& cutHalfEdge =
            result.mesh.halfEdges[
                halfEdgeId
            ];

        if (cutHalfEdge.face >= 0)
        {
            result.embeddedFaceIds.insert(
                cutHalfEdge.face
            );
        }

        if (cutHalfEdge.twin >= 0 &&
            cutHalfEdge.twin <
                static_cast<int>(
                    result.mesh.halfEdges.size()
                ))
        {
            const int twinFaceId =
                result.mesh.halfEdges[
                    cutHalfEdge.twin
                ].face;

            if (twinFaceId >= 0)
            {
                result.embeddedFaceIds.insert(
                    twinFaceId
                );
            }
        }
    }

    std::unordered_map<int, std::vector<int>>
        curveIdsByMeshVertex;

    for (const auto& entry :
         result.cutChainsByCurveId)
    {
        const int curveId =
            entry.first;

        const CutChain& cutChain =
            entry.second;

        for (int meshVertexId :
             cutChain.vertexIds)
        {
            std::vector<int>& connectedCurveIds =
                curveIdsByMeshVertex[
                    meshVertexId
                ];

            /*
                Avoid storing the same curve more than once
                if a chain happens to reference a vertex
                repeatedly.
            */
            if (
                std::find(
                    connectedCurveIds.begin(),
                    connectedCurveIds.end(),
                    curveId
                ) == connectedCurveIds.end()
            )
            {
                connectedCurveIds.push_back(
                    curveId
                );
            }
        }
    }

    for (const auto& entry :
         curveIdsByMeshVertex)
    {
        const int meshVertexId =
            entry.first;

        const std::vector<int>& connectedCurveIds =
            entry.second;

        /*
            A shared Curvenet node must belong to at
            least two different profile curves.
        */
        if (connectedCurveIds.size() < 2)
        {
            continue;
        }

        SharedCurvenetNode sharedNode;

        sharedNode.meshVertexId =
            meshVertexId;

        sharedNode.connectedCurveIds =
            connectedCurveIds;

        result.sharedCurvenetNodes.push_back(
            sharedNode
        );
    }

    result.success = true;

    return result;
}
