#include "CurvenetMeshCutter.h"

#include "CutPathMeshSplitter.h"
#include "CurvenetSharedNodeDetector.h"
#include "CurveMeshIntersector.h"
#include "GeometryUtils.h"

#include <algorithm>
#include <unordered_map>

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

            for (int meshVertexId :
                 cutChain.vertexIds)
            {
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

            sharedNode.connectedCurveIds =
                connectedCurveIds;

            result.sharedCurvenetNodes.push_back(
                sharedNode
            );
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

        cutPath.curveId =
            profileInput.curveId;

        cutPath.closed =
            profileInput.closed;

        cutPath.crossings =
            crossings;

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

    buildSharedCurvenetNodes(
        result
    );

    result.success = true;

    return result;
}
