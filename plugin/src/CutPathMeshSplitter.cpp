#include "CutPathMeshSplitter.h"
#include <algorithm>
#include "GeometryUtils.h"
#include <unordered_map>

namespace
{
    struct OrderedCutVertex
    {
        const CutVertex* cutVertex = nullptr;

        int originalIndex = -1;
    };

    struct ProcessedCut
    {
        Point3 position;

        int meshVertexId = -1;
    };

    int findProcessedCutVertex(
        const Point3& position,
        const std::vector<ProcessedCut>& processedCuts,
        double duplicateTolerance
    )
    {
        for (const ProcessedCut& processedCut :
             processedCuts)
        {
            const double distance =
                GeometryUtils::pointToPointDistance(
                    position,
                    processedCut.position
                );

            if (distance <= duplicateTolerance)
            {
                return processedCut.meshVertexId;
            }
        }

        return -1;
    }
}

CutPathSplitResult
CutPathMeshSplitter::apply(
    HalfEdgeMesh& mesh,
    const CutPath& cutPath,
    double duplicateTolerance
)
{
    CutPathSplitResult result;

    result.success = false;

    result.meshVertexIds.resize(
        cutPath.cutVertices.size(),
        -1
    );

    std::vector<OrderedCutVertex> orderedCutVertices;

    for (int cutVertexIndex = 0;
         cutVertexIndex <
             static_cast<int>(
                 cutPath.cutVertices.size()
             );
         ++cutVertexIndex)
    {
        OrderedCutVertex orderedCutVertex;

        orderedCutVertex.cutVertex =
            &cutPath.cutVertices[cutVertexIndex];

        orderedCutVertex.originalIndex =
            cutVertexIndex;

        orderedCutVertices.push_back(
            orderedCutVertex
        );
    }

    std::sort(
        orderedCutVertices.begin(),
        orderedCutVertices.end(),
        [](
            const OrderedCutVertex& first,
            const OrderedCutVertex& second
        )
        {
            return
                first.cutVertex->cutPathOrder <
                second.cutVertex->cutPathOrder;
        }
    );

    std::unordered_map<int, int> edgeRedirects;

    std::vector<ProcessedCut> processedCuts;

    for (const OrderedCutVertex& orderedCutVertex :
         orderedCutVertices)
    {
        if (orderedCutVertex.cutVertex == nullptr)
        {
            return result;
        }

        const CutVertex& cutVertex =
            *orderedCutVertex.cutVertex;

        const int existingMeshVertexId =
            findProcessedCutVertex(
                cutVertex.position,
                processedCuts,
                duplicateTolerance
            );

        if (existingMeshVertexId >= 0)
        {
            result.meshVertexIds[
                orderedCutVertex.originalIndex
            ] = existingMeshVertexId;

            continue;
        }

        int currentHalfEdgeId =
            cutVertex.sourceHalfEdgeId;

        const auto redirectIterator =
            edgeRedirects.find(
                cutVertex.sourceHalfEdgeId
            );

        if (redirectIterator !=
            edgeRedirects.end())
        {
            currentHalfEdgeId =
                redirectIterator->second;
        }

        if (currentHalfEdgeId < 0 ||
            currentHalfEdgeId >=
                static_cast<int>(mesh.halfEdges.size()))
        {
            return result;
        }

        int newMeshVertexId = -1;
        int remainingHalfEdgeId = -1;

        const HalfEdge& currentHalfEdge =
            mesh.halfEdges[currentHalfEdgeId];

        if (currentHalfEdge.twin < 0)
        {
            const BoundaryHalfEdgeSplitResult splitResult =
                mesh.splitBoundaryHalfEdge(
                    currentHalfEdgeId,
                    cutVertex.position
                );

            if (!splitResult.success)
            {
                return result;
            }

            newMeshVertexId =
                splitResult.newVertexId;

            remainingHalfEdgeId =
                splitResult.secondHalfEdgeId;
        }
        else
        {
            const InternalHalfEdgeSplitResult splitResult =
                mesh.splitInternalHalfEdge(
                    currentHalfEdgeId,
                    cutVertex.position
                );

            if (!splitResult.success)
            {
                return result;
            }

            newMeshVertexId =
                splitResult.newVertexId;

            remainingHalfEdgeId =
                splitResult.firstNewHalfEdgeId;
        }

        edgeRedirects[
            cutVertex.sourceHalfEdgeId
        ] = remainingHalfEdgeId;

        result.meshVertexIds[
            orderedCutVertex.originalIndex
        ] = newMeshVertexId;

        ProcessedCut processedCut;

        processedCut.position =
            cutVertex.position;

        processedCut.meshVertexId =
            newMeshVertexId;

        processedCuts.push_back(
            processedCut
        );
    }

    result.success = true;

    return result;
}
