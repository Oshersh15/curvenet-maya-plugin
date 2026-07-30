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

    result.cutChain.curveId =
        cutPath.curveId;

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

    for (const OrderedCutVertex& orderedCutVertex :
         orderedCutVertices)
    {
        const int originalIndex =
            orderedCutVertex.originalIndex;

        if (originalIndex < 0 ||
            originalIndex >=
                static_cast<int>(
                    result.meshVertexIds.size()
                ))
        {
            return result;
        }

        const int meshVertexId =
            result.meshVertexIds[
                originalIndex
            ];

        if (meshVertexId < 0)
        {
            return result;
        }

        result.cutChain.vertexIds.push_back(
            meshVertexId
        );
    }

    for (int intervalIndex = 0;
         intervalIndex <
             static_cast<int>(
                 cutPath.faceIntervalIds.size()
             );
         ++intervalIndex)
    {
        if (intervalIndex >=
            static_cast<int>(
                orderedCutVertices.size()
            ))
        {
            return result;
        }

        int nextOrderedIndex =
            intervalIndex + 1;

        if (nextOrderedIndex >=
            static_cast<int>(
                orderedCutVertices.size()
            ))
        {
            if (!cutPath.closed)
            {
                return result;
            }

            nextOrderedIndex = 0;
        }

        const int firstOriginalIndex =
            orderedCutVertices[
                intervalIndex
            ].originalIndex;

        const int secondOriginalIndex =
            orderedCutVertices[
                nextOrderedIndex
            ].originalIndex;

        const int firstVertexId =
            result.meshVertexIds[
                firstOriginalIndex
            ];

        const int secondVertexId =
            result.meshVertexIds[
                secondOriginalIndex
            ];

        if (firstVertexId < 0 ||
            secondVertexId < 0)
        {
            return result;
        }

        const int currentFaceId =
            mesh.findFaceContainingVertices(
                firstVertexId,
                secondVertexId
            );

        if (currentFaceId < 0)
        {
            return result;
        }

        const CutHalfEdgePairResult cutEdgeResult =
            createCutHalfEdges(
                mesh,
                firstVertexId,
                secondVertexId
            );

        if (!cutEdgeResult.success)
        {
            return result;
        }

        const bool inserted =
            insertCutHalfEdgesIntoFace(
                mesh,
                currentFaceId,
                cutEdgeResult.firstHalfEdgeId,
                cutEdgeResult.secondHalfEdgeId
            );

        if (!inserted)
        {
            return result;
        }

        result.cutChain.halfEdgeIds.push_back(
            cutEdgeResult.firstHalfEdgeId
        );
    }

    result.cutChain.closed = false;

    if (cutPath.closed)
    {
        if (result.cutChain.vertexIds.empty() ||
            result.cutChain.halfEdgeIds.empty())
        {
            return result;
        }

        const int firstVertexId =
            result.cutChain.vertexIds.front();

        const int lastVertexId =
            result.cutChain.vertexIds.back();

        const int closingHalfEdgeId =
            result.cutChain.halfEdgeIds.back();

        if (closingHalfEdgeId < 0 ||
            closingHalfEdgeId >=
                static_cast<int>(
                    mesh.halfEdges.size()
                ))
        {
            return result;
        }

        const HalfEdge& closingHalfEdge =
            mesh.halfEdges[
                closingHalfEdgeId
            ];

        if (closingHalfEdge.startVertex !=
                lastVertexId ||
            closingHalfEdge.endVertex !=
                firstVertexId)
        {
            return result;
        }

        result.cutChain.closed = true;
    }

    result.success = true;

    return result;
}

CutHalfEdgePairResult
CutPathMeshSplitter::createCutHalfEdges(
    HalfEdgeMesh& mesh,
    int firstVertexId,
    int secondVertexId
)
{
    CutHalfEdgePairResult result;

    if (firstVertexId < 0 ||
        firstVertexId >=
            static_cast<int>(mesh.vertices.size()) ||
        secondVertexId < 0 ||
        secondVertexId >=
            static_cast<int>(mesh.vertices.size()))
    {
        return result;
    }

    const int firstHalfEdgeId =
        static_cast<int>(
            mesh.halfEdges.size()
        );

    const int secondHalfEdgeId =
        firstHalfEdgeId + 1;

    HalfEdge firstHalfEdge;
    firstHalfEdge.startVertex = firstVertexId;
    firstHalfEdge.endVertex = secondVertexId;

    HalfEdge secondHalfEdge;
    secondHalfEdge.startVertex = secondVertexId;
    secondHalfEdge.endVertex = firstVertexId;

    firstHalfEdge.twin =
        secondHalfEdgeId;

    secondHalfEdge.twin =
        firstHalfEdgeId;

    mesh.halfEdges.push_back(
        firstHalfEdge
    );

    mesh.halfEdges.push_back(
        secondHalfEdge
    );

    result.success = true;
    result.firstHalfEdgeId =
        firstHalfEdgeId;
    result.secondHalfEdgeId =
        secondHalfEdgeId;

    return result;
}

bool CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
    HalfEdgeMesh& mesh,
    int faceId,
    int firstCutHalfEdgeId,
    int secondCutHalfEdgeId
)
{
    if (faceId < 0 ||
        faceId >= static_cast<int>(mesh.faces.size()))
    {
        return false;
    }

    if (firstCutHalfEdgeId < 0 ||
        firstCutHalfEdgeId >=
            static_cast<int>(mesh.halfEdges.size()) ||
        secondCutHalfEdgeId < 0 ||
        secondCutHalfEdgeId >=
            static_cast<int>(mesh.halfEdges.size()))
    {
        return false;
    }

    HalfEdge& firstCutHalfEdge =
        mesh.halfEdges[firstCutHalfEdgeId];

    HalfEdge& secondCutHalfEdge =
        mesh.halfEdges[secondCutHalfEdgeId];

    const int firstVertexId =
        firstCutHalfEdge.startVertex;

    const int secondVertexId =
        firstCutHalfEdge.endVertex;

    const int firstOutgoingHalfEdgeId =
        mesh.findOutgoingHalfEdgeInFace(
            faceId,
            firstVertexId
        );

    const int secondOutgoingHalfEdgeId =
        mesh.findOutgoingHalfEdgeInFace(
            faceId,
            secondVertexId
        );

    if (firstOutgoingHalfEdgeId < 0 ||
        secondOutgoingHalfEdgeId < 0)
    {
        return false;
    }

    const int previousFirstHalfEdgeId =
        mesh.findPreviousHalfEdgeInFace(
            faceId,
            firstOutgoingHalfEdgeId
        );

    const int previousSecondHalfEdgeId =
        mesh.findPreviousHalfEdgeInFace(
            faceId,
            secondOutgoingHalfEdgeId
        );

    if (previousFirstHalfEdgeId < 0 ||
        previousSecondHalfEdgeId < 0)
    {
        return false;
    }

    /*
        Preserve the original face on the loop containing
        firstVertex -> ... -> secondVertex -> firstVertex.
    */
    mesh.halfEdges[
        previousSecondHalfEdgeId
    ].next = secondCutHalfEdgeId;

    secondCutHalfEdge.next =
        firstOutgoingHalfEdgeId;

    /*
        The opposite boundary path becomes the new face.
    */
    mesh.halfEdges[
        previousFirstHalfEdgeId
    ].next = firstCutHalfEdgeId;

    firstCutHalfEdge.next =
        secondOutgoingHalfEdgeId;

    secondCutHalfEdge.face =
        faceId;

    mesh.faces[faceId].halfEdge =
        secondCutHalfEdgeId;

    const int newFaceId =
        static_cast<int>(
            mesh.faces.size()
        );

    Face newFace;
    newFace.halfEdge =
        firstCutHalfEdgeId;

    mesh.faces.push_back(newFace);

    /*
        Walk the newly created loop and assign every
        half-edge to the new face.
    */
    int currentHalfEdgeId =
        firstCutHalfEdgeId;

    do
    {
        if (currentHalfEdgeId < 0 ||
            currentHalfEdgeId >=
                static_cast<int>(
                    mesh.halfEdges.size()
                ))
        {
            return false;
        }

        mesh.halfEdges[
            currentHalfEdgeId
        ].face = newFaceId;

        currentHalfEdgeId =
            mesh.halfEdges[
                currentHalfEdgeId
            ].next;

    } while (
        currentHalfEdgeId !=
        firstCutHalfEdgeId
    );

    return true;
}
