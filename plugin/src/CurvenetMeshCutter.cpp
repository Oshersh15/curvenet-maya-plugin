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
    bool faceContainsPoint(
        const HalfEdgeMesh& mesh,
        int faceId,
        const Point3& point,
        double tolerance = 0.000001
    )
    {
        const std::vector<int> halfEdgeIds = mesh.traverseFace(faceId);

        if (halfEdgeIds.size() < 3)
        {
            return false;
        }

        const Point3& anchor = mesh.vertices[
            mesh.halfEdges[halfEdgeIds[0]].startVertex
        ].position;

        for (int index = 1;
             index + 1 < static_cast<int>(halfEdgeIds.size());
             ++index)
        {
            const Point3& first = mesh.vertices[
                mesh.halfEdges[halfEdgeIds[index]].startVertex
            ].position;
            const Point3& second = mesh.vertices[
                mesh.halfEdges[halfEdgeIds[index + 1]].startVertex
            ].position;
            const double ax = first.x - anchor.x;
            const double ay = first.y - anchor.y;
            const double az = first.z - anchor.z;
            const double bx = second.x - anchor.x;
            const double by = second.y - anchor.y;
            const double bz = second.z - anchor.z;
            const double px = point.x - anchor.x;
            const double py = point.y - anchor.y;
            const double pz = point.z - anchor.z;
            const double aa = ax * ax + ay * ay + az * az;
            const double ab = ax * bx + ay * by + az * bz;
            const double bb = bx * bx + by * by + bz * bz;
            const double ap = ax * px + ay * py + az * pz;
            const double bp = bx * px + by * py + bz * pz;
            const double denominator = aa * bb - ab * ab;

            if (std::abs(denominator) <= tolerance * tolerance)
            {
                continue;
            }

            const double firstWeight = (bb * ap - ab * bp) / denominator;
            const double secondWeight = (aa * bp - ab * ap) / denominator;

            if (firstWeight >= -tolerance &&
                secondWeight >= -tolerance &&
                firstWeight + secondWeight <= 1.0 + tolerance)
            {
                const double nx = ay * bz - az * by;
                const double ny = az * bx - ax * bz;
                const double nz = ax * by - ay * bx;
                const double normalLength = std::sqrt(
                    nx * nx + ny * ny + nz * nz
                );
                const double planeDistance = normalLength > tolerance
                    ? std::abs(px * nx + py * ny + pz * nz) /
                        normalLength
                    : std::numeric_limits<double>::infinity();

                if (planeDistance <= tolerance)
                {
                    return true;
                }
            }
        }

        return false;
    }

    void buildSharedCurvenetNodes(
        CurvenetCutResult& result,
        const std::unordered_set<int>* authoredNodeVertexIds,
        const std::vector<SharedCurvenetNode>& logicalEndpointNodes
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

            if (authoredNodeVertexIds != nullptr &&
                authoredNodeVertexIds->count(meshVertexId) == 0)
            {
                continue;
            }

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

        result.sharedCurvenetNodes.insert(
            result.sharedCurvenetNodes.end(),
            logicalEndpointNodes.begin(),
            logicalEndpointNodes.end()
        );
    }

    std::unordered_set<int> applyPhysicalEndpointJunctions(
        CurvenetCutResult& result,
        const std::vector<ProfileCutInput>& profileInputs,
        std::vector<SharedCurvenetNode>& logicalEndpointNodes
    )
    {
        std::unordered_set<int> authoredNodeVertexIds;

        struct EndpointRef
        {
            int curveId = -1;
            CurveEndpoint endpoint = CurveEndpoint::Start;
        };

        const auto endpointKey =
            [](int curveId, CurveEndpoint endpoint)
            {
                return curveId * 2 +
                    (endpoint == CurveEndpoint::Start ? 0 : 1);
            };

        std::unordered_map<int, int> parent;
        std::unordered_map<int, EndpointRef> endpoints;

        for (const ProfileCutInput& input : profileInputs)
        {
            const int startKey = endpointKey(
                input.curveId,
                CurveEndpoint::Start
            );
            const int endKey = endpointKey(
                input.curveId,
                CurveEndpoint::End
            );
            parent[startKey] = startKey;
            parent[endKey] = endKey;
            endpoints[startKey] = {
                input.curveId,
                CurveEndpoint::Start
            };
            endpoints[endKey] = {
                input.curveId,
                CurveEndpoint::End
            };
        }

        const auto findRoot = [&parent](int key)
        {
            int root = key;
            while (parent[root] != root)
            {
                root = parent[root];
            }
            return root;
        };

        const auto findInput =
            [&profileInputs](int curveId)
            -> const ProfileCutInput*
            {
                for (const ProfileCutInput& input : profileInputs)
                {
                    if (input.curveId == curveId)
                    {
                        return &input;
                    }
                }
                return nullptr;
            };

        for (const ProfileCutInput& input : profileInputs)
        {
            for (const ProfileCurveConnection& connection :
                 input.connections)
            {
                const ProfileCutInput* targetInput =
                    findInput(connection.targetCurveId);

                if (targetInput == nullptr ||
                    targetInput->sampledSegments.empty())
                {
                    continue;
                }

                CurveEndpoint targetEndpoint;
                const int lastSegmentId =
                    static_cast<int>(
                        targetInput->sampledSegments.size()
                    ) - 1;

                if (connection.targetSegmentId == 0 &&
                    connection.targetSegmentT <= 0.000001)
                {
                    targetEndpoint = CurveEndpoint::Start;
                }
                else if (
                    connection.targetSegmentId == lastSegmentId &&
                    connection.targetSegmentT >= 0.999999
                )
                {
                    targetEndpoint = CurveEndpoint::End;
                }
                else
                {
                    continue;
                }

                const int sourceKey = endpointKey(
                    input.curveId,
                    connection.endpoint
                );
                const int targetKey = endpointKey(
                    connection.targetCurveId,
                    targetEndpoint
                );
                const int sourceRoot = findRoot(sourceKey);
                const int targetRoot = findRoot(targetKey);

                if (sourceRoot != targetRoot)
                {
                    parent[targetRoot] = sourceRoot;
                }
            }
        }

        std::unordered_map<int, std::vector<EndpointRef>> groups;

        for (const auto& entry : endpoints)
        {
            groups[findRoot(entry.first)].push_back(entry.second);
        }

        for (const auto& groupEntry : groups)
        {
            const std::vector<EndpointRef>& group =
                groupEntry.second;

            if (group.size() < 2)
            {
                continue;
            }

            std::vector<int> boundaryVertexIds;
            std::vector<int> boundaryIndexByEndpoint;
            Point3 sharedPosition;
            bool hasSharedPosition = false;

            for (const EndpointRef& endpoint : group)
            {
                const auto chainIterator =
                    result.cutChainsByCurveId.find(endpoint.curveId);
                const ProfileCutInput* input =
                    findInput(endpoint.curveId);

                if (chainIterator ==
                        result.cutChainsByCurveId.end() ||
                    chainIterator->second.vertexIds.empty() ||
                    input == nullptr ||
                    input->sampledSegments.empty())
                {
                    boundaryIndexByEndpoint.push_back(-1);
                    continue;
                }

                const int boundaryVertexId =
                    endpoint.endpoint == CurveEndpoint::Start
                        ? chainIterator->second.vertexIds.front()
                        : chainIterator->second.vertexIds.back();

                auto boundaryIterator = std::find(
                    boundaryVertexIds.begin(),
                    boundaryVertexIds.end(),
                    boundaryVertexId
                );

                if (boundaryIterator == boundaryVertexIds.end())
                {
                    boundaryIndexByEndpoint.push_back(
                        static_cast<int>(boundaryVertexIds.size())
                    );
                    boundaryVertexIds.push_back(boundaryVertexId);
                }
                else
                {
                    boundaryIndexByEndpoint.push_back(
                        static_cast<int>(
                            boundaryIterator - boundaryVertexIds.begin()
                        )
                    );
                }

                if (!hasSharedPosition)
                {
                    sharedPosition =
                        endpoint.endpoint == CurveEndpoint::Start
                            ? input->sampledSegments.front().start
                            : input->sampledSegments.back().end;
                    hasSharedPosition = true;
                }
            }

            if (boundaryVertexIds.size() == 1)
            {
                authoredNodeVertexIds.insert(boundaryVertexIds.front());
                continue;
            }

            if (boundaryVertexIds.size() < 2 || !hasSharedPosition)
            {
                continue;
            }

            const auto preserveLogicalJunction = [&]()
            {
                SharedCurvenetNode logicalNode;
                logicalNode.position = sharedPosition;

                for (const EndpointRef& endpoint : group)
                {
                    if (std::find(
                            logicalNode.connectedCurveIds.begin(),
                            logicalNode.connectedCurveIds.end(),
                            endpoint.curveId
                        ) == logicalNode.connectedCurveIds.end())
                    {
                        logicalNode.connectedCurveIds.push_back(
                            endpoint.curveId
                        );
                    }
                }

                logicalEndpointNodes.push_back(std::move(logicalNode));
            };

            int seedFaceId = -1;
            std::vector<int> seedBoundaryIndices;
            bool seedContainsSharedPosition = false;

            for (int faceId = 0;
                 faceId < static_cast<int>(result.mesh.faces.size());
                 ++faceId)
            {
                std::vector<int> containedBoundaryIndices;

                for (int boundaryIndex = 0;
                     boundaryIndex < static_cast<int>(boundaryVertexIds.size());
                     ++boundaryIndex)
                {
                    if (result.mesh.findOutgoingHalfEdgeInFace(
                            faceId,
                            boundaryVertexIds[boundaryIndex]
                        ) >= 0)
                    {
                        containedBoundaryIndices.push_back(boundaryIndex);
                    }
                }

                const bool containsSharedPosition = faceContainsPoint(
                    result.mesh,
                    faceId,
                    sharedPosition
                );

                if (containedBoundaryIndices.size() < 2)
                {
                    continue;
                }

                if ((containsSharedPosition &&
                     !seedContainsSharedPosition) ||
                    (containsSharedPosition == seedContainsSharedPosition &&
                     containedBoundaryIndices.size() >
                         seedBoundaryIndices.size()))
                {
                    seedFaceId = faceId;
                    seedContainsSharedPosition = containsSharedPosition;
                    seedBoundaryIndices = std::move(
                        containedBoundaryIndices
                    );
                }
            }

            if (seedFaceId < 0 || seedBoundaryIndices.size() < 2)
            {
                /*
                    Authored endpoints can represent one logical node even
                    when cutting leaves their mesh vertices on different
                    faces. Preserve that authored junction without changing
                    the valid physical CutChains.
                */
                preserveLogicalJunction();
                continue;
            }

            Vertex sharedVertex;
            sharedVertex.position = sharedPosition;
            const int sharedVertexId =
                static_cast<int>(result.mesh.vertices.size());

            const HalfEdgeMesh meshBeforeJunction = result.mesh;
            result.mesh.vertices.push_back(sharedVertex);

            std::vector<int> seedBoundaryVertexIds;

            for (int boundaryIndex : seedBoundaryIndices)
            {
                seedBoundaryVertexIds.push_back(
                    boundaryVertexIds[boundaryIndex]
                );
            }

            const InteriorFaceSplitResult splitResult =
                result.mesh.splitFaceWithInteriorVertex(
                    seedFaceId,
                    sharedVertexId,
                    seedBoundaryVertexIds
                );

            if (!splitResult.success)
            {
                result.mesh = meshBeforeJunction;
                preserveLogicalJunction();
                continue;
            }

            std::vector<int> boundaryToSharedHalfEdgeIds(
                boundaryVertexIds.size(),
                -1
            );
            std::vector<int> sharedToBoundaryHalfEdgeIds(
                boundaryVertexIds.size(),
                -1
            );

            for (int seedIndex = 0;
                 seedIndex < static_cast<int>(seedBoundaryIndices.size());
                 ++seedIndex)
            {
                const int boundaryIndex = seedBoundaryIndices[seedIndex];
                boundaryToSharedHalfEdgeIds[boundaryIndex] =
                    splitResult.boundaryToInteriorHalfEdgeIds[seedIndex];
                sharedToBoundaryHalfEdgeIds[boundaryIndex] =
                    splitResult.interiorToBoundaryHalfEdgeIds[seedIndex];
            }

            bool connectedEveryBoundary = true;

            for (int boundaryIndex = 0;
                 boundaryIndex < static_cast<int>(boundaryVertexIds.size());
                 ++boundaryIndex)
            {
                if (boundaryToSharedHalfEdgeIds[boundaryIndex] >= 0)
                {
                    continue;
                }

                const int faceId = result.mesh.findFaceContainingVertices(
                    boundaryVertexIds[boundaryIndex],
                    sharedVertexId
                );
                const CutHalfEdgePairResult connector =
                    CutPathMeshSplitter::createCutHalfEdges(
                        result.mesh,
                        boundaryVertexIds[boundaryIndex],
                        sharedVertexId
                    );

                if (faceId < 0 || !connector.success ||
                    !CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
                        result.mesh,
                        faceId,
                        connector.firstHalfEdgeId,
                        connector.secondHalfEdgeId
                    ))
                {
                    connectedEveryBoundary = false;
                    break;
                }

                boundaryToSharedHalfEdgeIds[boundaryIndex] =
                    connector.firstHalfEdgeId;
                sharedToBoundaryHalfEdgeIds[boundaryIndex] =
                    connector.secondHalfEdgeId;
            }

            if (!connectedEveryBoundary)
            {
                result.mesh = meshBeforeJunction;
                preserveLogicalJunction();
                continue;
            }

            result.embeddedVertexIds.insert(sharedVertexId);
            authoredNodeVertexIds.insert(sharedVertexId);

            for (int halfEdgeId : boundaryToSharedHalfEdgeIds)
            {
                result.embeddedHalfEdgeIds.insert(halfEdgeId);
            }

            for (int halfEdgeId : sharedToBoundaryHalfEdgeIds)
            {
                result.embeddedHalfEdgeIds.insert(halfEdgeId);
            }

            for (int endpointIndex = 0;
                 endpointIndex < static_cast<int>(group.size());
                 ++endpointIndex)
            {
                const int boundaryIndex =
                    boundaryIndexByEndpoint[endpointIndex];

                if (boundaryIndex < 0)
                {
                    continue;
                }

                const EndpointRef& endpoint = group[endpointIndex];
                CutChain& chain =
                    result.cutChainsByCurveId.at(endpoint.curveId);

                EmbeddedCurvePoint point;
                point.meshVertexId = sharedVertexId;
                point.position = sharedPosition;

                if (endpoint.endpoint == CurveEndpoint::Start)
                {
                    chain.vertexIds.insert(
                        chain.vertexIds.begin(),
                        sharedVertexId
                    );
                    chain.points.insert(chain.points.begin(), point);
                    chain.halfEdgeIds.insert(
                        chain.halfEdgeIds.begin(),
                        sharedToBoundaryHalfEdgeIds[boundaryIndex]
                    );
                }
                else
                {
                    chain.vertexIds.push_back(sharedVertexId);
                    chain.points.push_back(point);
                    chain.halfEdgeIds.push_back(
                        boundaryToSharedHalfEdgeIds[boundaryIndex]
                    );
                }
            }
        }

        return authoredNodeVertexIds;
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
        result,
        nullptr,
        {}
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
    std::vector<SharedCurvenetNode> logicalEndpointNodes;
    const std::unordered_set<int> authoredNodeVertexIds =
        applyPhysicalEndpointJunctions(
        result,
        profileInputs,
        logicalEndpointNodes
    );

    const bool hasExplicitEndpointConnections =
        std::any_of(
            profileInputs.begin(),
            profileInputs.end(),
            [](const ProfileCutInput& input)
            {
                return !input.connections.empty();
            }
        );

    buildSharedCurvenetNodes(
        result,
        hasExplicitEndpointConnections
            ? &authoredNodeVertexIds
            : nullptr,
        logicalEndpointNodes
    );

    result.success = true;

    return result;
}
