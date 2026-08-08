#include "CutPathMeshSplitter.h"
#include <algorithm>
#include "GeometryUtils.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>


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

    std::vector<int> findExistingHalfEdgePath(
        const HalfEdgeMesh& mesh,
        int startVertexId,
        int endVertexId,
        int maximumDepth
    )
    {
        std::vector<int> emptyPath;

        if (startVertexId < 0 ||
            startVertexId >=
                static_cast<int>(
                    mesh.vertices.size()
                ) ||
            endVertexId < 0 ||
            endVertexId >=
                static_cast<int>(
                    mesh.vertices.size()
                ) ||
            maximumDepth <= 0)
        {
            return emptyPath;
        }

        struct QueueEntry
        {
            int vertexId = -1;
            int depth = 0;
        };

        std::queue<QueueEntry> queue;
        std::unordered_set<int> visitedVertices;
        std::unordered_map<int, int> predecessorVertexByVertex;
        std::unordered_map<int, int> predecessorHalfEdgeByVertex;

        queue.push({startVertexId, 0});
        visitedVertices.insert(startVertexId);

        while (!queue.empty())
        {
            const QueueEntry current =
                queue.front();
            queue.pop();

            if (current.depth >= maximumDepth)
            {
                continue;
            }

            for (int halfEdgeId = 0;
                 halfEdgeId <
                     static_cast<int>(
                         mesh.halfEdges.size()
                     );
                 ++halfEdgeId)
            {
                const HalfEdge& halfEdge =
                    mesh.halfEdges[
                        halfEdgeId
                    ];

                if (halfEdge.startVertex !=
                    current.vertexId)
                {
                    continue;
                }

                const int nextVertexId =
                    halfEdge.endVertex;

                if (nextVertexId < 0 ||
                    nextVertexId >=
                        static_cast<int>(
                            mesh.vertices.size()
                        ) ||
                    visitedVertices.find(
                        nextVertexId
                    ) != visitedVertices.end())
                {
                    continue;
                }

                visitedVertices.insert(
                    nextVertexId
                );

                predecessorVertexByVertex[
                    nextVertexId
                ] = current.vertexId;

                predecessorHalfEdgeByVertex[
                    nextVertexId
                ] = halfEdgeId;

                if (nextVertexId == endVertexId)
                {
                    std::vector<int> path;
                    int walkedVertexId =
                        endVertexId;

                    while (walkedVertexId !=
                           startVertexId)
                    {
                        const auto halfEdgeIterator =
                            predecessorHalfEdgeByVertex
                                .find(walkedVertexId);

                        const auto vertexIterator =
                            predecessorVertexByVertex
                                .find(walkedVertexId);

                        if (halfEdgeIterator ==
                                predecessorHalfEdgeByVertex
                                    .end() ||
                            vertexIterator ==
                                predecessorVertexByVertex
                                    .end())
                        {
                            return emptyPath;
                        }

                        path.push_back(
                            halfEdgeIterator->second
                        );

                        walkedVertexId =
                            vertexIterator->second;
                    }

                    std::reverse(
                        path.begin(),
                        path.end()
                    );

                    return path;
                }

                queue.push(
                    {
                        nextVertexId,
                        current.depth + 1
                    }
                );
            }
        }

        return emptyPath;
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

    std::vector<OrderedCutVertex>
        orderedCutVertices;

    for (int cutVertexIndex = 0;
         cutVertexIndex <
             static_cast<int>(
                 cutPath.cutVertices.size()
             );
         ++cutVertexIndex)
    {
        OrderedCutVertex orderedCutVertex;

        orderedCutVertex.cutVertex =
            &cutPath.cutVertices[
                cutVertexIndex
            ];

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
            if (first.cutVertex == nullptr)
            {
                return false;
            }

            if (second.cutVertex == nullptr)
            {
                return true;
            }

            return
                first.cutVertex->cutPathOrder <
                second.cutVertex->cutPathOrder;
        }
    );

    const int cutVertexCount =
        static_cast<int>(
            orderedCutVertices.size()
        );

    const int expectedIntervalCount =
        cutPath.closed
            ? cutVertexCount
            : cutVertexCount - 1;

    if (cutVertexCount <
            (cutPath.closed ? 3 : 2))
    {
        result.failure =
            CutPathSplitFailure::
                InvalidFaceInterval;

        return result;
    }

    if (!cutPath.faceIntervalIds.empty() &&
        static_cast<int>(
            cutPath.faceIntervalIds.size()
        ) != expectedIntervalCount)
    {
        result.failure =
            CutPathSplitFailure::
                InvalidFaceInterval;

        return result;
    }

    std::unordered_map<int, int>
        edgeRedirects;

    std::vector<ProcessedCut>
        processedCuts;

    /*
        Creates or reuses one mesh vertex only when
        the current interval needs it.

        This avoids creating every future cut vertex
        before earlier intervals divide their faces.
    */
    const auto resolveMeshVertex =
        [&](
            const OrderedCutVertex&
                orderedCutVertex
        ) -> int
        {
            if (
                orderedCutVertex.cutVertex ==
                nullptr
            )
            {
                result.failure =
                    CutPathSplitFailure::
                        NullCutVertex;

                return -1;
            }

            const int originalIndex =
                orderedCutVertex.originalIndex;

            if (originalIndex < 0 ||
                originalIndex >=
                    static_cast<int>(
                        result
                            .meshVertexIds
                            .size()
                    ))
            {
                result.failure =
                    CutPathSplitFailure::
                        InvalidMeshVertex;

                return -1;
            }

            /*
                This vertex may already have been
                resolved for the preceding interval.
            */
            if (result.meshVertexIds[
                    originalIndex
                ] >= 0)
            {
                return result.meshVertexIds[
                    originalIndex
                ];
            }

            const CutVertex& cutVertex =
                *orderedCutVertex.cutVertex;

            /*
                A higher-level Curvenet operation may
                already have identified this crossing
                as an existing shared mesh vertex.
            */
            if (
                cutVertex
                    .existingMeshVertexId >= 0
            )
            {
                if (
                    cutVertex
                        .existingMeshVertexId >=
                    static_cast<int>(
                        mesh.vertices.size()
                    )
                )
                {
                    result.failure =
                        CutPathSplitFailure::
                            InvalidExistingMeshVertex;

                    return -1;
                }

                result.meshVertexIds[
                    originalIndex
                ] =
                    cutVertex
                        .existingMeshVertexId;

                ProcessedCut processedCut;

                processedCut.position =
                    cutVertex.position;

                processedCut.meshVertexId =
                    cutVertex
                        .existingMeshVertexId;

                processedCuts.push_back(
                    processedCut
                );

                return
                    cutVertex
                        .existingMeshVertexId;
            }

            const int existingMeshVertexId =
                findProcessedCutVertex(
                    cutVertex.position,
                    processedCuts,
                    duplicateTolerance
                );

            if (existingMeshVertexId >= 0)
            {
                result.meshVertexIds[
                    originalIndex
                ] =
                    existingMeshVertexId;

                return existingMeshVertexId;
            }

            int currentHalfEdgeId =
                cutVertex.sourceHalfEdgeId;

            const auto redirectIterator =
                edgeRedirects.find(
                    cutVertex
                        .sourceHalfEdgeId
                );

            if (
                redirectIterator !=
                edgeRedirects.end()
            )
            {
                currentHalfEdgeId =
                    redirectIterator->second;
            }

            if (currentHalfEdgeId < 0 ||
                currentHalfEdgeId >=
                    static_cast<int>(
                        mesh.halfEdges.size()
                    ))
            {
                result.failure =
                    CutPathSplitFailure::
                        InvalidHalfEdge;

                return -1;
            }

            int newMeshVertexId = -1;
            int remainingHalfEdgeId = -1;

            const HalfEdge currentHalfEdge =
                mesh.halfEdges[
                    currentHalfEdgeId
                ];

            if (currentHalfEdge.twin < 0)
            {
                const BoundaryHalfEdgeSplitResult
                    splitResult =
                        mesh.splitBoundaryHalfEdge(
                            currentHalfEdgeId,
                            cutVertex.position
                        );

                if (!splitResult.success)
                {
                    result.failure =
                        CutPathSplitFailure::
                            BoundarySplitFailed;

                    return -1;
                }

                newMeshVertexId =
                    splitResult.newVertexId;

                remainingHalfEdgeId =
                    splitResult
                        .secondHalfEdgeId;
            }
            else
            {
                const InternalHalfEdgeSplitResult
                    splitResult =
                        mesh.splitInternalHalfEdge(
                            currentHalfEdgeId,
                            cutVertex.position
                        );

                if (!splitResult.success)
                {
                    result.failure =
                        CutPathSplitFailure::
                            InternalSplitFailed;

                    return -1;
                }

                newMeshVertexId =
                    splitResult.newVertexId;

                remainingHalfEdgeId =
                    splitResult
                        .firstNewHalfEdgeId;
            }

            edgeRedirects[
                cutVertex.sourceHalfEdgeId
            ] = remainingHalfEdgeId;

            result.meshVertexIds[
                originalIndex
            ] = newMeshVertexId;

            ProcessedCut processedCut;

            processedCut.position =
                cutVertex.position;

            processedCut.meshVertexId =
                newMeshVertexId;

            processedCuts.push_back(
                processedCut
            );

            return newMeshVertexId;
        };

    /*
        Some callers use the splitter only to resolve
        ordered CutVertices and do not provide face
        intervals. Preserve that supported behaviour.
    */
    if (cutPath.faceIntervalIds.empty())
    {
        for (const OrderedCutVertex& orderedCutVertex :
             orderedCutVertices)
        {
            const int meshVertexId =
                resolveMeshVertex(
                    orderedCutVertex
                );

            if (meshVertexId < 0)
            {
                return result;
            }

            result.cutChain.vertexIds.push_back(
                meshVertexId
            );

            EmbeddedCurvePoint point;

            point.meshVertexId =
                meshVertexId;

            point.curveSegmentId =
                orderedCutVertex
                    .cutVertex
                    ->curveSegmentId;

            point.curveSegmentT =
                orderedCutVertex
                    .cutVertex
                    ->curveSegmentT;

            point.position =
                orderedCutVertex
                    .cutVertex
                    ->position;

            result.cutChain
                .points
                .push_back(
                    point
                );

            result.cutChain.cutVertexIndices.push_back(
                orderedCutVertex.originalIndex
            );
        }

        result.cutChain.closed = false;
        result.success = true;

        return result;
    }

    /*
        Resolve each interval’s endpoints and divide
        its current face immediately.

        Future endpoints are not created until their
        interval is reached.
    */
    for (int intervalIndex = 0;
         intervalIndex <
             expectedIntervalCount;
         ++intervalIndex)
    {
        const int firstOrderedIndex =
            intervalIndex;

        int secondOrderedIndex =
            intervalIndex + 1;

        if (secondOrderedIndex >=
            cutVertexCount)
        {
            if (!cutPath.closed)
            {
                result.failure =
                    CutPathSplitFailure::
                        InvalidFaceInterval;

                return result;
            }

            secondOrderedIndex = 0;
        }

        const OrderedCutVertex&
            firstOrderedCutVertex =
                orderedCutVertices[
                    firstOrderedIndex
                ];

        const OrderedCutVertex&
            secondOrderedCutVertex =
                orderedCutVertices[
                    secondOrderedIndex
                ];

        const int firstVertexId =
            resolveMeshVertex(
                firstOrderedCutVertex
            );

        if (firstVertexId < 0)
        {
            return result;
        }

        const int secondVertexId =
            resolveMeshVertex(
                secondOrderedCutVertex
            );

        if (secondVertexId < 0)
        {
            return result;
        }

        /*
            A zero-length interval can occur when
            two CutVertices resolve to the same
            existing shared vertex.
        */
        if (firstVertexId == secondVertexId)
        {
            continue;
        }

        int existingHalfEdgeId =
            mesh.findHalfEdge(
                firstVertexId,
                secondVertexId
            );

        if (existingHalfEdgeId < 0)
        {
            const int reverseHalfEdgeId =
                mesh.findHalfEdge(
                    secondVertexId,
                    firstVertexId
                );

            if (reverseHalfEdgeId >= 0)
            {
                existingHalfEdgeId =
                    reverseHalfEdgeId;
            }
        }

        if (existingHalfEdgeId >= 0)
        {
            result.cutChain
                .halfEdgeIds
                .push_back(
                    existingHalfEdgeId
                );

            continue;
        }

        int currentFaceId = -1;

        /*
            Prefer the face interval computed from the original
            curve/mesh crossings when that face still contains
            both resolved endpoints in the already-cut mesh.

            Free-drawn Curvenet segments can pass through small
            corner cases where previous cuts change the first
            matching face returned by a global search. The interval
            hint preserves the intended local face when it remains
            topologically valid.
        */
        if (intervalIndex >= 0 &&
            intervalIndex <
                static_cast<int>(
                    cutPath.faceIntervalIds.size()
                ))
        {
            const int hintedFaceId =
                cutPath.faceIntervalIds[
                    intervalIndex
                ];

            if (hintedFaceId >= 0 &&
                hintedFaceId <
                    static_cast<int>(
                        mesh.faces.size()
                    ) &&
                mesh.findOutgoingHalfEdgeInFace(
                    hintedFaceId,
                    firstVertexId
                ) >= 0 &&
                mesh.findOutgoingHalfEdgeInFace(
                    hintedFaceId,
                    secondVertexId
                ) >= 0)
            {
                currentFaceId =
                    hintedFaceId;
            }
        }

        if (currentFaceId < 0)
        {
            currentFaceId =
                mesh.findFaceContainingVertices(
                    firstVertexId,
                    secondVertexId
                );
        }

        if (currentFaceId < 0)
        {
            const std::vector<int> existingPath =
                findExistingHalfEdgePath(
                    mesh,
                    firstVertexId,
                    secondVertexId,
                    8
                );

            if (!existingPath.empty())
            {
                result.cutChain
                    .halfEdgeIds
                    .insert(
                        result.cutChain
                            .halfEdgeIds
                            .end(),
                        existingPath.begin(),
                        existingPath.end()
                    );

                continue;
            }

            result.failure =
                CutPathSplitFailure::
                    VerticesNotOnSameFace;

            result.failedIntervalIndex =
                intervalIndex;

            result.failedFirstVertexId =
                firstVertexId;

            result.failedSecondVertexId =
                secondVertexId;

            return result;
        }

        const CutHalfEdgePairResult
            cutEdgeResult =
                createCutHalfEdges(
                    mesh,
                    firstVertexId,
                    secondVertexId
                );

        if (!cutEdgeResult.success)
        {
            result.failure =
                CutPathSplitFailure::
                    CreateCutHalfEdgesFailed;

            return result;
        }

        const bool inserted =
            insertCutHalfEdgesIntoFace(
                mesh,
                currentFaceId,
                cutEdgeResult
                    .firstHalfEdgeId,
                cutEdgeResult
                    .secondHalfEdgeId
            );

        if (!inserted)
        {
            result.failure =
                CutPathSplitFailure::
                    InsertCutHalfEdgesFailed;

            return result;
        }

        result.cutChain
            .halfEdgeIds
            .push_back(
                cutEdgeResult
                    .firstHalfEdgeId
            );
    }

    /*
        Store the CutChain vertices in profile
        traversal order, independent of the order
        in which they were resolved.
    */
    for (const OrderedCutVertex&
         orderedCutVertex :
         orderedCutVertices)
    {
        const int originalIndex =
            orderedCutVertex.originalIndex;

        if (originalIndex < 0 ||
            originalIndex >=
                static_cast<int>(
                    result
                        .meshVertexIds
                        .size()
                ))
        {
            result.failure =
                CutPathSplitFailure::
                    InvalidMeshVertex;

            return result;
        }

        const int meshVertexId =
            result.meshVertexIds[
                originalIndex
            ];

        if (meshVertexId < 0)
        {
            result.failure =
                CutPathSplitFailure::
                    InvalidMeshVertex;

            return result;
        }

        result.cutChain
            .vertexIds
            .push_back(
                meshVertexId
            );

        EmbeddedCurvePoint point;

        point.meshVertexId =
            meshVertexId;

        point.curveSegmentId =
            orderedCutVertex
                .cutVertex
                ->curveSegmentId;

        point.curveSegmentT =
            orderedCutVertex
                .cutVertex
                ->curveSegmentT;

        point.position =
            orderedCutVertex
                .cutVertex
                ->position;

        result.cutChain
            .points
            .push_back(
                point
            );
    }

    result.cutChain.closed = false;

    if (cutPath.closed)
    {
        if (
            result.cutChain
                .vertexIds
                .empty() ||
            result.cutChain
                .halfEdgeIds
                .empty()
        )
        {
            result.failure =
                CutPathSplitFailure::
                    InvalidClosingHalfEdge;

            return result;
        }

        const int firstVertexId =
            result.cutChain
                .vertexIds
                .front();

        const int lastVertexId =
            result.cutChain
                .vertexIds
                .back();

        const int closingHalfEdgeId =
            result.cutChain
                .halfEdgeIds
                .back();

        if (
            closingHalfEdgeId < 0 ||
            closingHalfEdgeId >=
                static_cast<int>(
                    mesh.halfEdges.size()
                )
        )
        {
            result.failure =
                CutPathSplitFailure::
                    InvalidClosingHalfEdge;

            return result;
        }

        const HalfEdge& closingHalfEdge =
            mesh.halfEdges[
                closingHalfEdgeId
            ];

        if (
            closingHalfEdge.startVertex !=
                lastVertexId ||
            closingHalfEdge.endVertex !=
                firstVertexId
        )
        {
            result.failure =
                CutPathSplitFailure::
                    ClosingEdgeMismatch;

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
