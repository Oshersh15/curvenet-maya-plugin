#include "CurvenetMeshCutter.h"

#include "CutPathMeshSplitter.h"
#include "CurvenetSharedNodeDetector.h"
#include "CurveMeshIntersector.h"
#include "GeometryUtils.h"

#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <limits>

namespace
{
    void buildSharedCurvenetNodes(
        CurvenetCutResult& result
    )
    {
        result.sharedCurvenetNodes.clear();

        std::unordered_map<
            int,
            std::vector<int>
        > curveIdsByMeshVertex;

        for (const auto& entry :
             result.cutChainsByCurveId)
        {
            const int curveId =
                entry.first;

            const CutChain& cutChain =
                entry.second;

            for (const EmbeddedCurvePoint& point :
                 cutChain.points)
            {
                const int meshVertexId =
                    point.meshVertexId;

                if (meshVertexId < 0)
                {
                    continue;
                }

                std::vector<int>& connectedCurveIds =
                    curveIdsByMeshVertex[
                        meshVertexId
                    ];

                if (
                    std::find(
                        connectedCurveIds.begin(),
                        connectedCurveIds.end(),
                        curveId
                    ) ==
                    connectedCurveIds.end()
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

            if (connectedCurveIds.size() < 2)
            {
                continue;
            }

            SharedCurvenetNode sharedNode;

            sharedNode.meshVertexId =
                meshVertexId;

            if (meshVertexId >= 0 &&
                meshVertexId <
                    static_cast<int>(
                        result.mesh.vertices.size()
                    ))
            {
                sharedNode.position =
                    result.mesh
                        .vertices[meshVertexId]
                        .position;
            }

            sharedNode.connectedCurveIds =
                connectedCurveIds;

            result.sharedCurvenetNodes.push_back(
                sharedNode
            );
        }
    }

    void applyExplicitProfileConnections(
        CurvenetCutResult& result,
        const std::vector<ProfileCutInput>& profileInputs
    )
    {
        std::unordered_map<int, int> parentByEndpointKey;

        const auto findProfileInput =
            [&profileInputs](
                int curveId
            ) -> const ProfileCutInput*
            {
                for (const ProfileCutInput& input :
                     profileInputs)
                {
                    if (input.curveId == curveId)
                    {
                        return &input;
                    }
                }

                return nullptr;
            };

        const auto endpointKey =
            [](int curveId, CurveEndpoint endpoint)
            {
                return curveId * 2 +
                    (
                        endpoint == CurveEndpoint::Start
                            ? 0
                            : 1
                    );
            };

        auto findRoot =
            [&parentByEndpointKey](int key)
            {
                int currentKey = key;

                while (
                    parentByEndpointKey.find(currentKey) !=
                        parentByEndpointKey.end() &&
                    parentByEndpointKey[currentKey] !=
                        currentKey)
                {
                    currentKey =
                        parentByEndpointKey[currentKey];
                }

                int rootKey = currentKey;
                currentKey = key;

                while (
                    parentByEndpointKey.find(currentKey) !=
                        parentByEndpointKey.end() &&
                    parentByEndpointKey[currentKey] !=
                        currentKey)
                {
                    const int nextKey =
                        parentByEndpointKey[currentKey];

                    parentByEndpointKey[currentKey] =
                        rootKey;

                    currentKey = nextKey;
                }

                return rootKey;
            };

        auto ensureEndpoint =
            [&parentByEndpointKey](int key)
            {
                if (parentByEndpointKey.find(key) ==
                    parentByEndpointKey.end())
                {
                    parentByEndpointKey[key] = key;
                }
            };

        auto uniteEndpoints =
            [&](
                int firstKey,
                int secondKey
            )
            {
                ensureEndpoint(firstKey);
                ensureEndpoint(secondKey);

                const int firstRoot =
                    findRoot(firstKey);

                const int secondRoot =
                    findRoot(secondKey);

                if (firstRoot != secondRoot)
                {
                    parentByEndpointKey[secondRoot] =
                        firstRoot;
                }
            };

        const auto sourceEndpointPosition =
            [](
                const ProfileCutInput& profileInput,
                CurveEndpoint endpoint,
                const CutChain& fallbackChain,
                int fallbackPointIndex
            )
            {
                if (!profileInput.sampledSegments.empty())
                {
                    if (endpoint == CurveEndpoint::Start)
                    {
                        return profileInput
                            .sampledSegments
                            .front()
                            .start;
                    }

                    return profileInput
                        .sampledSegments
                        .back()
                        .end;
                }

                return fallbackChain
                    .points[fallbackPointIndex]
                    .position;
            };

        const auto targetEndpointKeyForConnection =
            [&](
                const ProfileCurveConnection& connection
            ) -> int
            {
                const ProfileCutInput* targetInput =
                    findProfileInput(
                        connection.targetCurveId
                    );

                if (targetInput == nullptr)
                {
                    return -1;
                }

                const int lastSegmentId =
                    static_cast<int>(
                        targetInput
                            ->sampledSegments
                            .size()
                    ) - 1;

                const double tolerance = 0.000001;

                if (connection.targetSegmentId == 0 &&
                    connection.targetSegmentT <= tolerance)
                {
                    return endpointKey(
                        connection.targetCurveId,
                        CurveEndpoint::Start
                    );
                }

                if (connection.targetSegmentId ==
                        lastSegmentId &&
                    connection.targetSegmentT >=
                        1.0 - tolerance)
                {
                    return endpointKey(
                        connection.targetCurveId,
                        CurveEndpoint::End
                    );
                }

                return -1;
            };

        for (const ProfileCutInput& profileInput :
             profileInputs)
        {
            for (const ProfileCurveConnection& connection :
                 profileInput.connections)
            {
                const int sourceKey =
                    endpointKey(
                        profileInput.curveId,
                        connection.endpoint
                    );

                const int targetKey =
                    targetEndpointKeyForConnection(
                        connection
                    );

                if (targetKey >= 0)
                {
                    uniteEndpoints(
                        sourceKey,
                        targetKey
                    );
                }
            }
        }

        std::unordered_map<int, int>
            sharedMeshVertexIdByRootKey;

        for (const ProfileCutInput& profileInput :
             profileInputs)
        {
            auto endpointChainIterator =
                result.cutChainsByCurveId.find(
                    profileInput.curveId
                );

            if (endpointChainIterator ==
                result.cutChainsByCurveId.end())
            {
                continue;
            }

            for (const ProfileCurveConnection& connection :
                 profileInput.connections)
            {
                CutChain& endpointChain =
                    endpointChainIterator->second;

                if (endpointChain.points.empty())
                {
                    continue;
                }

                const int endpointPointIndex =
                    connection.endpoint ==
                            CurveEndpoint::Start
                        ? 0
                        : static_cast<int>(
                              endpointChain.points.size()
                          ) - 1;

                if (endpointPointIndex < 0 ||
                    endpointPointIndex >=
                        static_cast<int>(
                            endpointChain.points.size()
                        ))
                {
                    continue;
                }

                const int sourceEndpointKey =
                    endpointKey(
                        profileInput.curveId,
                        connection.endpoint
                    );

                ensureEndpoint(sourceEndpointKey);

                const int rootEndpointKey =
                    findRoot(sourceEndpointKey);

                const Point3 sharedPosition =
                    sourceEndpointPosition(
                        profileInput,
                        connection.endpoint,
                        endpointChain,
                        endpointPointIndex
                    );

                auto targetChainIterator =
                    result.cutChainsByCurveId.find(
                        connection.targetCurveId
                    );

                if (targetChainIterator ==
                    result.cutChainsByCurveId.end())
                {
                    continue;
                }

                CutChain& targetChain =
                    targetChainIterator->second;

                int closestPointIndex = -1;
                double closestDistance =
                    std::numeric_limits<double>::max();

                for (int pointIndex = 0;
                     pointIndex <
                         static_cast<int>(
                             targetChain.points.size()
                         );
                     ++pointIndex)
                {
                    const EmbeddedCurvePoint& point =
                        targetChain.points[
                            pointIndex
                        ];

                    if (point.curveSegmentId !=
                        connection.targetSegmentId)
                    {
                        continue;
                    }

                    const double distance =
                        std::abs(
                            point.curveSegmentT -
                            connection.targetSegmentT
                        );

                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        closestPointIndex = pointIndex;
                    }
                }

                int sharedMeshVertexId = -1;

                const auto existingSharedIterator =
                    sharedMeshVertexIdByRootKey.find(
                        rootEndpointKey
                    );

                if (existingSharedIterator !=
                    sharedMeshVertexIdByRootKey.end())
                {
                    sharedMeshVertexId =
                        existingSharedIterator->second;
                }
                else
                {
                    /*
                    Authored endpoints can be one logical node even when
                    separate cuts produced different mesh vertices.
                    */
                    Vertex sharedVertex;

                    sharedVertex.position =
                        sharedPosition;

                    sharedMeshVertexId =
                        static_cast<int>(
                            result.mesh.vertices.size()
                        );

                    result.mesh.vertices.push_back(
                        sharedVertex
                    );

                    sharedMeshVertexIdByRootKey[
                        rootEndpointKey
                    ] = sharedMeshVertexId;
                }

                endpointChain.points[
                    endpointPointIndex
                ].meshVertexId =
                    sharedMeshVertexId;

                if (endpointPointIndex <
                    static_cast<int>(
                        endpointChain.vertexIds.size()
                    ))
                {
                    endpointChain.vertexIds[
                        endpointPointIndex
                    ] = sharedMeshVertexId;
                }

                if (closestPointIndex >= 0)
                {
                    targetChain.points[
                        closestPointIndex
                    ].meshVertexId =
                        sharedMeshVertexId;

                    if (closestPointIndex <
                        static_cast<int>(
                            targetChain.vertexIds.size()
                        ))
                    {
                        targetChain.vertexIds[
                            closestPointIndex
                        ] = sharedMeshVertexId;
                    }
                }
                else
                {
                    EmbeddedCurvePoint insertedPoint;

                    insertedPoint.meshVertexId =
                        sharedMeshVertexId;

                    insertedPoint.curveSegmentId =
                        connection.targetSegmentId;

                    insertedPoint.curveSegmentT =
                        connection.targetSegmentT;

                    insertedPoint.position =
                        sharedPosition;

                    int insertionIndex = 0;

                    while (
                        insertionIndex <
                        static_cast<int>(
                            targetChain.points.size()
                        ))
                    {
                        const EmbeddedCurvePoint& point =
                            targetChain.points[
                                insertionIndex
                            ];

                        if (point.curveSegmentId >
                                insertedPoint
                                    .curveSegmentId ||
                            (
                                point.curveSegmentId ==
                                    insertedPoint
                                        .curveSegmentId &&
                                point.curveSegmentT >
                                    insertedPoint
                                        .curveSegmentT
                            ))
                        {
                            break;
                        }

                        ++insertionIndex;
                    }

                    targetChain.points.insert(
                        targetChain.points.begin()
                            + insertionIndex,
                        insertedPoint
                    );

                    targetChain.vertexIds.insert(
                        targetChain.vertexIds.begin()
                            + insertionIndex,
                        sharedMeshVertexId
                    );
                }
            }
        }
    }
}

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

    buildSharedCurvenetNodes(
        result
    );


    result.success = true;

    return result;
}

CurvenetCutResult CurvenetMeshCutter::apply(
    const HalfEdgeMesh& inputMesh,
    const std::vector<ProfileCutInput>& profileInputs,
    double crossingTolerance,
    double duplicateTolerance
)
{
    CurvenetCutResult result;

    result.mesh = inputMesh;

    for (const ProfileCutInput& profileInput :
         profileInputs)
    {
        std::vector<CutCrossing> crossings =
            CurveMeshIntersector::findAllCrossings(
                profileInput.curveId,
                profileInput.sampledSegments,
                result.mesh,
                crossingTolerance,
                duplicateTolerance
            );

        CutPath cutPath;

        cutPath.curveId = profileInput.curveId;

        cutPath.closed = profileInput.closed;

        cutPath.crossings = crossings;

        int connectedStartMeshVertexId = -1;
        int connectedEndMeshVertexId = -1;

        for (const ProfileCurveConnection& connection :
             profileInput.connections)
        {
            const auto curveIterator =
                result.embeddedVerticesByCurveAndSegment.find(
                    connection.targetCurveId
                );

            if (curveIterator ==
                result.embeddedVerticesByCurveAndSegment.end())
            {
                continue;
            }

            const auto segmentIterator =
                curveIterator->second.find(
                    connection.targetSegmentId
                );

            if (segmentIterator ==
                curveIterator->second.end())
            {
                continue;
            }

            const std::vector<EmbeddedSegmentVertex>&
                candidates =
                    segmentIterator->second;

            if (candidates.empty())
            {
                continue;
            }

            const EmbeddedSegmentVertex*
                closestCandidate = nullptr;

            double closestDistance =
                std::numeric_limits<double>::max();

            for (const EmbeddedSegmentVertex& candidate :
                 candidates)
            {
                const double distance =
                    std::abs(
                        candidate.segmentT -
                        connection.targetSegmentT
                    );

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestCandidate = &candidate;
                }
            }

            if (closestCandidate == nullptr ||
                closestCandidate->meshVertexId < 0)
            {
                continue;
            }

            if (connection.endpoint ==
                CurveEndpoint::Start)
            {
                connectedStartMeshVertexId =
                    closestCandidate->meshVertexId;
            }
            else
            {
                connectedEndMeshVertexId =
                    closestCandidate->meshVertexId;
            }
        }

        /*
            A profile passing extremely close to a mesh corner can
            detect both incident edges as separate crossings.

            When both crossings are close to their shared mesh
            vertex, they represent one corner crossing rather than
            a real face interval. Collapse them and snap the
            crossing to the existing mesh vertex.
        */
        std::vector<int> snappedMeshVertexIds(
            cutPath.crossings.size(),
            -1
        );

        bool collapsedCornerCrossing = true;

        while (collapsedCornerCrossing)
        {
            collapsedCornerCrossing = false;

            for (int crossingIndex = 0;
                 crossingIndex + 1 <
                     static_cast<int>(
                         cutPath.crossings.size()
                     );
                 ++crossingIndex)
            {
                const CutCrossing& firstCrossing =
                    cutPath.crossings[
                        crossingIndex
                    ];

                const CutCrossing& secondCrossing =
                    cutPath.crossings[
                        crossingIndex + 1
                    ];

                if (firstCrossing.halfEdgeId < 0 ||
                    firstCrossing.halfEdgeId >=
                        static_cast<int>(
                            result.mesh.halfEdges.size()
                        ) ||
                    secondCrossing.halfEdgeId < 0 ||
                    secondCrossing.halfEdgeId >=
                        static_cast<int>(
                            result.mesh.halfEdges.size()
                        ))
                {
                    continue;
                }

                const double crossingDistance =
                    GeometryUtils::pointToPointDistance(
                        firstCrossing.position,
                        secondCrossing.position
                    );

                if (crossingDistance >
                    crossingTolerance)
                {
                    continue;
                }

                const HalfEdge& firstHalfEdge =
                    result.mesh.halfEdges[
                        firstCrossing.halfEdgeId
                    ];

                const HalfEdge& secondHalfEdge =
                    result.mesh.halfEdges[
                        secondCrossing.halfEdgeId
                    ];

                int sharedVertexId = -1;

                if (firstHalfEdge.startVertex ==
                        secondHalfEdge.startVertex ||
                    firstHalfEdge.startVertex ==
                        secondHalfEdge.endVertex)
                {
                    sharedVertexId =
                        firstHalfEdge.startVertex;
                }
                else if (
                    firstHalfEdge.endVertex ==
                        secondHalfEdge.startVertex ||
                    firstHalfEdge.endVertex ==
                        secondHalfEdge.endVertex
                )
                {
                    sharedVertexId =
                        firstHalfEdge.endVertex;
                }

                if (sharedVertexId < 0 ||
                    sharedVertexId >=
                        static_cast<int>(
                            result.mesh.vertices.size()
                        ))
                {
                    continue;
                }

                const Point3& sharedPosition =
                    result.mesh.vertices[
                        sharedVertexId
                    ].position;

                const double firstDistance =
                    GeometryUtils::pointToPointDistance(
                        firstCrossing.position,
                        sharedPosition
                    );

                const double secondDistance =
                    GeometryUtils::pointToPointDistance(
                        secondCrossing.position,
                        sharedPosition
                    );

                if (firstDistance >
                        crossingTolerance ||
                    secondDistance >
                        crossingTolerance)
                {
                    continue;
                }

                cutPath.crossings[
                    crossingIndex
                ].position = sharedPosition;

                snappedMeshVertexIds[
                    crossingIndex
                ] = sharedVertexId;

                cutPath.crossings.erase(
                    cutPath.crossings.begin()
                    + crossingIndex + 1
                );

                snappedMeshVertexIds.erase(
                    snappedMeshVertexIds.begin()
                    + crossingIndex + 1
                );

                collapsedCornerCrossing = true;
                break;
            }
        }

        /*
            Snapping crossings to existing mesh corners can make
            two previously distinct crossings occupy the same
            position.

            Remove those duplicates from the crossing list itself
            so crossings, CutVertices and face intervals remain
            one-to-one.
        */
        std::vector<CutCrossing> uniqueCrossings;

        for (const CutCrossing& crossing :
             cutPath.crossings)
        {
            bool duplicateFound = false;

            for (const CutCrossing& existingCrossing :
                 uniqueCrossings)
            {
                const double distance =
                    GeometryUtils::pointToPointDistance(
                        crossing.position,
                        existingCrossing.position
                    );

                if (distance <= duplicateTolerance)
                {
                    duplicateFound = true;
                    break;
                }
            }

            if (!duplicateFound)
            {
                uniqueCrossings.push_back(
                    crossing
                );
            }
        }

        cutPath.crossings =
            uniqueCrossings;

        cutPath.cutVertices =
            CurveMeshIntersector::buildCutVertices(
                cutPath.crossings,
                duplicateTolerance
            );

        cutPath.faceIntervalIds =
            CurveMeshIntersector::deriveFaceIntervals(
                cutPath,
                result.mesh
            );

        /*
            Two very close crossing detections can occur around
            one mesh corner. They should be collapsed only when
            they produce an invalid face interval.

            This is deliberately narrower than increasing the
            global duplicate tolerance, which merged valid
            neighbouring crossings on Tube B.
        */
        bool removedInvalidNearDuplicate = true;

        while (removedInvalidNearDuplicate)
        {
            removedInvalidNearDuplicate = false;

            for (int intervalIndex = 0;
                 intervalIndex <
                     static_cast<int>(
                         cutPath.faceIntervalIds.size()
                     );
                 ++intervalIndex)
            {
                if (cutPath.faceIntervalIds[
                        intervalIndex
                    ] >= 0)
                {
                    continue;
                }

                int secondCrossingIndex =
                    intervalIndex + 1;

                if (secondCrossingIndex >=
                    static_cast<int>(
                        cutPath.crossings.size()
                    ))
                {
                    if (!cutPath.closed)
                    {
                        continue;
                    }

                    secondCrossingIndex = 0;
                }

                if (intervalIndex < 0 ||
                    intervalIndex >=
                        static_cast<int>(
                            cutPath.crossings.size()
                        ) ||
                    secondCrossingIndex < 0 ||
                    secondCrossingIndex >=
                        static_cast<int>(
                            cutPath.crossings.size()
                        ))
                {
                    continue;
                }

                const double crossingDistance =
                    GeometryUtils::pointToPointDistance(
                        cutPath.crossings[
                            intervalIndex
                        ].position,
                        cutPath.crossings[
                            secondCrossingIndex
                        ].position
                    );

                if (crossingDistance >
                    crossingTolerance)
                {
                    continue;
                }

                cutPath.crossings.erase(
                    cutPath.crossings.begin()
                    + secondCrossingIndex
                );

                cutPath.cutVertices =
                    CurveMeshIntersector::
                        buildCutVertices(
                            cutPath.crossings,
                            duplicateTolerance
                        );

                cutPath.faceIntervalIds =
                    CurveMeshIntersector::
                        deriveFaceIntervals(
                            cutPath,
                            result.mesh
                        );

                /*
                    A profile with no usable intervals has not been
                    embedded and must not produce an empty CutChain.
                */
                const int minimumCutVertexCount =
                    cutPath.closed
                        ? 3
                        : 2;

                const int expectedIntervalCount =
                    cutPath.closed
                        ? static_cast<int>(
                              cutPath.cutVertices.size()
                          )
                        : static_cast<int>(
                              cutPath.cutVertices.size()
                          ) - 1;

                if (
                    static_cast<int>(
                        cutPath.cutVertices.size()
                    ) < minimumCutVertexCount ||
                    static_cast<int>(
                        cutPath.faceIntervalIds.size()
                    ) != expectedIntervalCount
                )
                {
                    result.attemptedCutPaths.push_back(
                        cutPath
                    );

                    result.failedCurveId =
                        cutPath.curveId;

                    return result;
                }

                removedInvalidNearDuplicate =
                    true;

                break;
            }
        }

        cutPath.influencedFaceIds =
            CurveMeshIntersector::collectUniqueFaces(
                cutPath.faceIntervalIds
            );

        cutPath.influencedVertexIds =
            result.mesh.collectUniqueVerticesFromFaces(
                cutPath.influencedFaceIds
            );

        /*
            Reuse an existing mesh vertex when crossing cleanup
            has snapped a CutVertex exactly onto that vertex.
        */
        for (CutVertex& cutVertex :
             cutPath.cutVertices)
        {
            for (int meshVertexId = 0;
                 meshVertexId <
                     static_cast<int>(
                         result.mesh.vertices.size()
                     );
                 ++meshVertexId)
            {
                const double distance =
                    GeometryUtils::pointToPointDistance(
                        cutVertex.position,
                        result.mesh.vertices[
                            meshVertexId
                        ].position
                    );

                if (distance <= duplicateTolerance)
                {
                    cutVertex.existingMeshVertexId =
                        meshVertexId;

                    break;
                }
            }
        }

        /*
            Reuse any existing embedded Curvenet vertex
            reached by the incoming profile.

            This checks every CutVertex, not only profile
            endpoints, because closed profiles and crossing
            profiles may meet existing chains internally.
        */
        for (CutVertex& cutVertex :
             cutPath.cutVertices)
        {
            const std::optional<int> sharedVertexId =
                CurvenetSharedNodeDetector::
                    findSharedMeshVertex(
                        cutVertex,
                        result,
                        duplicateTolerance
                    );

            if (sharedVertexId.has_value())
            {
                cutVertex.existingMeshVertexId =
                    sharedVertexId.value();
            }
        }

        /*
            Apply explicit endpoint-to-curve connections after
            all CutVertex rebuilding and general reuse checks.

            This ensures the detected profile connection takes
            precedence for the incoming profile endpoint.
        */
        if (!cutPath.closed &&
            !cutPath.cutVertices.empty())
        {
            int firstEndpointIndex = -1;
            int lastEndpointIndex = -1;

            for (int cutVertexIndex = 0;
                 cutVertexIndex <
                     static_cast<int>(
                         cutPath.cutVertices.size()
                     );
                 ++cutVertexIndex)
            {
                const int cutPathOrder =
                    cutPath.cutVertices[
                        cutVertexIndex
                    ].cutPathOrder;

                if (firstEndpointIndex < 0 ||
                    cutPathOrder <
                        cutPath.cutVertices[
                            firstEndpointIndex
                        ].cutPathOrder)
                {
                    firstEndpointIndex =
                        cutVertexIndex;
                }

                if (lastEndpointIndex < 0 ||
                    cutPathOrder >
                        cutPath.cutVertices[
                            lastEndpointIndex
                        ].cutPathOrder)
                {
                    lastEndpointIndex =
                        cutVertexIndex;
                }
            }

            if (firstEndpointIndex >= 0 &&
                connectedStartMeshVertexId >= 0)
            {
                cutPath.cutVertices[
                    firstEndpointIndex
                ].existingMeshVertexId =
                    connectedStartMeshVertexId;
            }

            if (lastEndpointIndex >= 0 &&
                connectedEndMeshVertexId >= 0)
            {
                cutPath.cutVertices[
                    lastEndpointIndex
                ].existingMeshVertexId =
                    connectedEndMeshVertexId;
            }
        }

        result.attemptedCutPaths.push_back(
            cutPath
        );

        CutPathSplitResult profileResult =
            CutPathMeshSplitter::apply(
                result.mesh,
                cutPath,
                duplicateTolerance
            );

        if (!profileResult.success)
        {
            result.failedCurveId =
                cutPath.curveId;

            result.failedSplitReason =
                profileResult.failure;

            result.failedIntervalIndex =
                profileResult.failedIntervalIndex;

            result.failedFirstVertexId =
                profileResult.failedFirstVertexId;

            result.failedSecondVertexId =
                profileResult.failedSecondVertexId;

            return result;
        }

        result.profileResults.push_back(
            profileResult
        );

        result.cutChainsByCurveId[
            profileInput.curveId
        ] = profileResult.cutChain;

        std::unordered_map<
            int,
            std::vector<EmbeddedSegmentVertex>
        >& verticesBySegment =
            result
                .embeddedVerticesByCurveAndSegment[
                    profileInput.curveId
                ];

        const int mappedCount =
            std::min(
                static_cast<int>(
                    cutPath.cutVertices.size()
                ),
                static_cast<int>(
                    profileResult
                        .cutChain
                        .vertexIds
                        .size()
                )
            );

        for (int cutVertexIndex = 0;
             cutVertexIndex < mappedCount;
             ++cutVertexIndex)
        {
            const CutVertex& cutVertex =
                cutPath.cutVertices[
                    cutVertexIndex
                ];

            EmbeddedSegmentVertex embeddedVertex;

            embeddedVertex.segmentT =
                cutVertex.curveSegmentT;

            embeddedVertex.meshVertexId =
                profileResult
                    .cutChain
                    .vertexIds[
                        cutVertexIndex
                    ];

            verticesBySegment[
                cutVertex.curveSegmentId
            ].push_back(
                embeddedVertex
            );
        }

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

    /*
    Cutting must finish first so authored endpoint relationships can be
    applied to the final cut chains and their generated mesh vertices.
    */
    applyExplicitProfileConnections(
        result,
        profileInputs
    );

    buildSharedCurvenetNodes(
        result
    );

    result.success = true;

    return result;
}
