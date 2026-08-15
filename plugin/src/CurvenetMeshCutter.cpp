/* Embeds sampled profile curves into an evolving half-edge mesh. */

#include "CurvenetMeshCutter.h"

#include "CutPathMeshSplitter.h"
#include "CurvenetSharedNodeDetector.h"
#include "CurveMeshIntersector.h"
#include "GeometryUtils.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
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

    std::vector<int> findFacesContainingPoint(
        const HalfEdgeMesh& mesh,
        const Point3& point,
        double tolerance
    )
    {
        std::vector<int> faceIds;

        for (int faceId = 0;
             faceId < static_cast<int>(mesh.faces.size());
             ++faceId)
        {
            if (faceContainsPoint(mesh, faceId, point, tolerance))
            {
                faceIds.push_back(faceId);
            }
        }

        if (!faceIds.empty())
        {
            return faceIds;
        }

        /*
            Maya's projected polyline can sit a few floating-point units away
            from a non-planar polygon. Select the nearest polygon as a
            deterministic fallback instead of treating the sample as being
            off the surface and reverting to a 3D edge-distance test.
        */
        int nearestFaceId = -1;
        double nearestDistanceSquared =
            std::numeric_limits<double>::infinity();

        for (int faceId = 0;
             faceId < static_cast<int>(mesh.faces.size());
             ++faceId)
        {
            const std::vector<int> halfEdgeIds =
                mesh.traverseFace(faceId);

            if (halfEdgeIds.size() < 3)
            {
                continue;
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
                const Point3 firstDirection =
                    GeometryUtils::subtract(first, anchor);
                const Point3 secondDirection =
                    GeometryUtils::subtract(second, anchor);
                const Point3 offset =
                    GeometryUtils::subtract(point, anchor);
                const double firstFirst = GeometryUtils::dot(
                    firstDirection,
                    firstDirection
                );
                const double firstSecond = GeometryUtils::dot(
                    firstDirection,
                    secondDirection
                );
                const double secondSecond = GeometryUtils::dot(
                    secondDirection,
                    secondDirection
                );
                const double offsetFirst = GeometryUtils::dot(
                    offset,
                    firstDirection
                );
                const double offsetSecond = GeometryUtils::dot(
                    offset,
                    secondDirection
                );
                const double denominator =
                    firstFirst * secondSecond -
                    firstSecond * firstSecond;

                if (std::abs(denominator) <= 1e-18)
                {
                    continue;
                }

                double firstWeight = (
                    secondSecond * offsetFirst -
                    firstSecond * offsetSecond
                ) / denominator;
                double secondWeight = (
                    firstFirst * offsetSecond -
                    firstSecond * offsetFirst
                ) / denominator;

                firstWeight = GeometryUtils::clamp(
                    firstWeight,
                    0.0,
                    1.0
                );
                secondWeight = GeometryUtils::clamp(
                    secondWeight,
                    0.0,
                    1.0 - firstWeight
                );

                Point3 closest = GeometryUtils::addScaled(
                    anchor,
                    firstDirection,
                    firstWeight
                );
                closest = GeometryUtils::addScaled(
                    closest,
                    secondDirection,
                    secondWeight
                );
                const Point3 difference =
                    GeometryUtils::subtract(point, closest);
                const double distanceSquared = GeometryUtils::dot(
                    difference,
                    difference
                );

                if (distanceSquared < nearestDistanceSquared)
                {
                    nearestDistanceSquared = distanceSquared;
                    nearestFaceId = faceId;
                }
            }
        }

        if (nearestFaceId >= 0)
        {
            faceIds.push_back(nearestFaceId);
        }

        return faceIds;
    }

    std::vector<int> findNeighbourFacesContainingPoint(
        const HalfEdgeMesh& mesh,
        const std::vector<std::vector<int>>& faceIdsByVertex,
        int seedFaceId,
        const Point3& point,
        double tolerance
    )
    {
        std::unordered_set<int> candidateFaceIds;

        if (seedFaceId < 0 ||
            seedFaceId >= static_cast<int>(mesh.faces.size()))
        {
            return {};
        }

        candidateFaceIds.insert(seedFaceId);

        for (int halfEdgeId : mesh.traverseFace(seedFaceId))
        {
            if (halfEdgeId < 0 ||
                halfEdgeId >= static_cast<int>(mesh.halfEdges.size()))
            {
                continue;
            }

            const HalfEdge& halfEdge = mesh.halfEdges[halfEdgeId];
            const int twinHalfEdgeId = halfEdge.twin;

            if (twinHalfEdgeId >= 0 &&
                twinHalfEdgeId < static_cast<int>(mesh.halfEdges.size()))
            {
                const int twinFaceId =
                    mesh.halfEdges[twinHalfEdgeId].face;

                if (twinFaceId >= 0)
                {
                    candidateFaceIds.insert(twinFaceId);
                }
            }

            if (halfEdge.startVertex < 0 ||
                halfEdge.startVertex >= static_cast<int>(
                    faceIdsByVertex.size()
                ))
            {
                continue;
            }

            for (int incidentFaceId :
                 faceIdsByVertex[halfEdge.startVertex])
            {
                if (incidentFaceId >= 0)
                {
                    candidateFaceIds.insert(incidentFaceId);
                }
            }
        }

        std::vector<int> faceIds;

        for (int candidateFaceId : candidateFaceIds)
        {
            if (faceContainsPoint(
                    mesh,
                    candidateFaceId,
                    point,
                    tolerance
                ))
            {
                faceIds.push_back(candidateFaceId);
            }
        }

        std::sort(faceIds.begin(), faceIds.end());
        return faceIds;
    }

    int findHalfEdgeBetweenFaces(
        const HalfEdgeMesh& mesh,
        int firstFaceId,
        int secondFaceId
    )
    {
        for (int halfEdgeId : mesh.traverseFace(firstFaceId))
        {
            if (halfEdgeId < 0 ||
                halfEdgeId >= static_cast<int>(mesh.halfEdges.size()))
            {
                continue;
            }

            const int twinId = mesh.halfEdges[halfEdgeId].twin;

            if (twinId >= 0 &&
                twinId < static_cast<int>(mesh.halfEdges.size()) &&
                mesh.halfEdges[twinId].face == secondFaceId)
            {
                return halfEdgeId;
            }
        }

        return -1;
    }

    int findSharedVertexBetweenFaces(
        const HalfEdgeMesh& mesh,
        int firstFaceId,
        int secondFaceId
    )
    {
        const std::vector<int> firstHalfEdgeIds =
            mesh.traverseFace(firstFaceId);
        const std::vector<int> secondHalfEdgeIds =
            mesh.traverseFace(secondFaceId);

        for (int firstHalfEdgeId : firstHalfEdgeIds)
        {
            const int firstVertexId =
                mesh.halfEdges[firstHalfEdgeId].startVertex;

            for (int secondHalfEdgeId : secondHalfEdgeIds)
            {
                if (mesh.halfEdges[secondHalfEdgeId].startVertex ==
                    firstVertexId)
                {
                    return firstVertexId;
                }
            }
        }

        return -1;
    }

    Point3 faceNormal(
        const HalfEdgeMesh& mesh,
        int faceId
    )
    {
        const std::vector<int> halfEdgeIds = mesh.traverseFace(faceId);

        if (halfEdgeIds.size() < 3)
        {
            return {};
        }

        const Point3& anchor = mesh.vertices[
            mesh.halfEdges[halfEdgeIds[0]].startVertex
        ].position;
        Point3 normal;

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
            const Point3 firstDirection = GeometryUtils::subtract(
                first,
                anchor
            );
            const Point3 secondDirection = GeometryUtils::subtract(
                second,
                anchor
            );
            normal.x += firstDirection.y * secondDirection.z -
                firstDirection.z * secondDirection.y;
            normal.y += firstDirection.z * secondDirection.x -
                firstDirection.x * secondDirection.z;
            normal.z += firstDirection.x * secondDirection.y -
                firstDirection.y * secondDirection.x;
        }

        const double length = GeometryUtils::length(normal);

        if (length <= 1e-12)
        {
            return {};
        }

        return {normal.x / length, normal.y / length, normal.z / length};
    }

    bool findUnfoldedSurfaceCrossing(
        const HalfEdgeMesh& mesh,
        int halfEdgeId,
        int startFaceId,
        int endFaceId,
        const PolylineSegment& segment,
        double& curveT,
        double& edgeT,
        Point3& position
    )
    {
        const HalfEdge& halfEdge = mesh.halfEdges[halfEdgeId];
        const Point3& edgeStart =
            mesh.vertices[halfEdge.startVertex].position;
        const Point3& edgeEnd =
            mesh.vertices[halfEdge.endVertex].position;
        const Point3 edgeDirection = GeometryUtils::subtract(
            edgeEnd,
            edgeStart
        );
        const double edgeLength = GeometryUtils::length(edgeDirection);

        if (edgeLength <= 1e-12)
        {
            return false;
        }

        const Point3 edgeAxis = {
            edgeDirection.x / edgeLength,
            edgeDirection.y / edgeLength,
            edgeDirection.z / edgeLength
        };
        const Point3 startNormal = faceNormal(mesh, startFaceId);
        const Point3 endNormal = faceNormal(mesh, endFaceId);
        Point3 startAcross = {
            startNormal.y * edgeAxis.z - startNormal.z * edgeAxis.y,
            startNormal.z * edgeAxis.x - startNormal.x * edgeAxis.z,
            startNormal.x * edgeAxis.y - startNormal.y * edgeAxis.x
        };
        Point3 endAcross = {
            endNormal.y * edgeAxis.z - endNormal.z * edgeAxis.y,
            endNormal.z * edgeAxis.x - endNormal.x * edgeAxis.z,
            endNormal.x * edgeAxis.y - endNormal.y * edgeAxis.x
        };

        const Point3 startOffset = GeometryUtils::subtract(
            segment.start,
            edgeStart
        );
        const Point3 endOffset = GeometryUtils::subtract(
            segment.end,
            edgeStart
        );
        const double startX = GeometryUtils::dot(startOffset, edgeAxis);
        const double endX = GeometryUtils::dot(endOffset, edgeAxis);
        const double startY = GeometryUtils::dot(startOffset, startAcross);
        double endY = GeometryUtils::dot(endOffset, endAcross);

        if (startY * endY > 0.0)
        {
            endY = -endY;
        }

        const double denominator = startY - endY;

        if (std::abs(denominator) <= 1e-12)
        {
            return false;
        }

        curveT = startY / denominator;
        const double crossingX =
            startX + (endX - startX) * curveT;
        edgeT = crossingX / edgeLength;

        if (curveT < -0.0001 || curveT > 1.0001 ||
            edgeT < -0.0001 || edgeT > 1.0001)
        {
            return false;
        }

        curveT = GeometryUtils::clamp(curveT, 0.0, 1.0);
        edgeT = GeometryUtils::clamp(edgeT, 0.0, 1.0);
        position = GeometryUtils::addScaled(
            edgeStart,
            edgeDirection,
            edgeT
        );
        return true;
    }

    struct SurfaceTrackedPath
    {
        std::vector<CutCrossing> crossings;
        std::vector<int> faceIntervalIds;
        bool reachedEndFace = false;
    };

    SurfaceTrackedPath buildSurfaceTrackedPath(
        int curveId,
        const std::vector<PolylineSegment>& sampledSegments,
        const HalfEdgeMesh& mesh,
        const std::vector<std::vector<int>>& faceIdsByVertex,
        bool closed,
        double crossingTolerance,
        double duplicateTolerance
    )
    {
        SurfaceTrackedPath path;
        const double surfaceTolerance =
            std::max(duplicateTolerance * 10.0, 0.000001);
        int currentFaceId = -1;

        for (int segmentId = 0;
             segmentId < static_cast<int>(sampledSegments.size());
             ++segmentId)
        {
            const PolylineSegment& segment = sampledSegments[segmentId];
            std::vector<int> startFaceIds;

            if (currentFaceId < 0)
            {
                startFaceIds = findFacesContainingPoint(
                    mesh,
                    segment.start,
                    surfaceTolerance
                );

                if (!startFaceIds.empty())
                {
                    currentFaceId = startFaceIds.front();
                }
            }

            if (currentFaceId >= 0)
            {
                startFaceIds = {currentFaceId};
            }

            std::vector<int> endFaceIds =
                findNeighbourFacesContainingPoint(
                    mesh,
                    faceIdsByVertex,
                    currentFaceId,
                    segment.end,
                    surfaceTolerance
                );

            if (endFaceIds.empty())
            {
                endFaceIds = findFacesContainingPoint(
                    mesh,
                    segment.end,
                    surfaceTolerance
                );
            }

            if (std::find(
                    endFaceIds.begin(),
                    endFaceIds.end(),
                    currentFaceId
                ) != endFaceIds.end())
            {
                continue;
            }

            int bestHalfEdgeId = -1;
            double bestCurveT = 0.0;
            double bestEdgeT = 0.0;
            Point3 bestPosition;
            int bestEndFaceId = -1;

            for (int startFaceId : startFaceIds)
            {
                if (currentFaceId >= 0 && startFaceId != currentFaceId)
                {
                    continue;
                }

                for (int endFaceId : endFaceIds)
                {
                    const int halfEdgeId = findHalfEdgeBetweenFaces(
                        mesh,
                        startFaceId,
                        endFaceId
                    );

                    if (halfEdgeId < 0)
                    {
                        continue;
                    }

                    double candidateCurveT = 0.0;
                    double candidateEdgeT = 0.0;
                    Point3 candidatePosition;

                    const bool foundCrossing = findUnfoldedSurfaceCrossing(
                            mesh,
                            halfEdgeId,
                            startFaceId,
                            endFaceId,
                            segment,
                            candidateCurveT,
                            candidateEdgeT,
                            candidatePosition
                        );

                    if (foundCrossing)
                    {
                        bestHalfEdgeId = halfEdgeId;
                        bestCurveT = candidateCurveT;
                        bestEdgeT = candidateEdgeT;
                        bestPosition = candidatePosition;
                        bestEndFaceId = endFaceId;
                        break;
                    }
                }

                if (bestHalfEdgeId >= 0)
                {
                    break;
                }
            }

            if (bestHalfEdgeId < 0)
            {
                for (int endFaceId : endFaceIds)
                {
                    const int sharedVertexId =
                        findSharedVertexBetweenFaces(
                            mesh,
                            currentFaceId,
                            endFaceId
                        );

                    if (sharedVertexId < 0)
                    {
                        continue;
                    }

                    const Point3& sharedPosition =
                        mesh.vertices[sharedVertexId].position;
                    const ClosestPointResult closest =
                        GeometryUtils::closestPointOnSegment(
                            sharedPosition,
                            segment.start,
                            segment.end
                        );

                    if (GeometryUtils::pointToPointDistance(
                            sharedPosition,
                            closest.point
                        ) > crossingTolerance)
                    {
                        continue;
                    }

                    for (int halfEdgeId :
                         mesh.traverseFace(currentFaceId))
                    {
                        const HalfEdge& halfEdge =
                            mesh.halfEdges[halfEdgeId];

                        if (halfEdge.startVertex == sharedVertexId ||
                            halfEdge.endVertex == sharedVertexId)
                        {
                            bestHalfEdgeId = halfEdgeId;
                            bestCurveT = closest.t;
                            bestEdgeT =
                                halfEdge.startVertex == sharedVertexId
                                    ? 0.0
                                    : 1.0;
                            bestPosition = sharedPosition;
                            bestEndFaceId = endFaceId;
                            break;
                        }
                    }

                    if (bestHalfEdgeId >= 0)
                    {
                        break;
                    }
                }
            }

            if (bestHalfEdgeId < 0)
            {
                continue;
            }

            const int crossedFromFaceId = currentFaceId;
            currentFaceId = bestEndFaceId;

            CutCrossing crossing;
            crossing.curveId = curveId;
            crossing.curveSegmentId = segmentId;
            crossing.curveSegmentT = bestCurveT;
            crossing.halfEdgeId = bestHalfEdgeId;
            crossing.faceId = mesh.halfEdges[bestHalfEdgeId].face;
            crossing.meshEdgeT = bestEdgeT;
            crossing.position = bestPosition;

            if (path.crossings.empty() ||
                GeometryUtils::pointToPointDistance(
                    path.crossings.back().position,
                    crossing.position
                ) > duplicateTolerance)
            {
                if (!path.crossings.empty())
                {
                    path.faceIntervalIds.push_back(
                        crossedFromFaceId
                    );
                }

                path.crossings.push_back(crossing);
            }
        }

        if (closed && !path.crossings.empty())
        {
            path.faceIntervalIds.push_back(currentFaceId);
        }

        if (!sampledSegments.empty())
        {
            std::vector<int> finalFaceIds =
                findNeighbourFacesContainingPoint(
                    mesh,
                    faceIdsByVertex,
                    currentFaceId,
                    sampledSegments.back().end,
                    surfaceTolerance
                );

            if (finalFaceIds.empty())
            {
                finalFaceIds = findFacesContainingPoint(
                    mesh,
                    sampledSegments.back().end,
                    surfaceTolerance
                );
            }

            path.reachedEndFace = std::find(
                finalFaceIds.begin(),
                finalFaceIds.end(),
                currentFaceId
            ) != finalFaceIds.end();
        }

        return path;
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
        std::vector<SharedCurvenetNode>& logicalEndpointNodes,
        double surfaceTolerance
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

            /*
                A projected node can lie on a mesh edge. Reuse or split that
                edge first, then connect each incident CutChain through its
                adjacent face instead of forcing the node into one face.
            */
            int edgeSharedVertexId = -1;
            const HalfEdgeMesh meshBeforeEdgeJunction = result.mesh;

            for (int vertexId = 0;
                 vertexId < static_cast<int>(result.mesh.vertices.size());
                 ++vertexId)
            {
                if (GeometryUtils::pointToPointDistance(
                        sharedPosition,
                        result.mesh.vertices[vertexId].position
                    ) <= surfaceTolerance)
                {
                    edgeSharedVertexId = vertexId;
                    break;
                }
            }

            if (edgeSharedVertexId < 0)
            {
                for (int halfEdgeId = 0;
                     halfEdgeId < static_cast<int>(result.mesh.halfEdges.size());
                     ++halfEdgeId)
                {
                    const HalfEdge& edge = result.mesh.halfEdges[halfEdgeId];

                    if (edge.twin >= 0 && halfEdgeId > edge.twin)
                    {
                        continue;
                    }

                    const Point3& start =
                        result.mesh.vertices[edge.startVertex].position;
                    const Point3& end =
                        result.mesh.vertices[edge.endVertex].position;
                    const ClosestPointResult closest =
                        GeometryUtils::closestPointOnSegment(
                            sharedPosition,
                            start,
                            end
                        );

                    if (closest.t <= 0.000001 || closest.t >= 0.999999 ||
                        GeometryUtils::pointToPointDistance(
                            sharedPosition,
                            closest.point
                        ) > surfaceTolerance)
                    {
                        continue;
                    }

                    if (edge.twin < 0)
                    {
                        const BoundaryHalfEdgeSplitResult split =
                            result.mesh.splitBoundaryHalfEdge(
                                halfEdgeId,
                                sharedPosition
                            );
                        edgeSharedVertexId = split.success
                            ? split.newVertexId
                            : -1;
                    }
                    else
                    {
                        const InternalHalfEdgeSplitResult split =
                            result.mesh.splitInternalHalfEdge(
                                halfEdgeId,
                                sharedPosition
                            );
                        edgeSharedVertexId = split.success
                            ? split.newVertexId
                            : -1;
                    }

                    break;
                }
            }

            if (edgeSharedVertexId >= 0)
            {
                std::vector<int> boundaryToShared(
                    boundaryVertexIds.size(),
                    -1
                );
                std::vector<int> sharedToBoundary(
                    boundaryVertexIds.size(),
                    -1
                );
                bool connected = true;

                for (int boundaryIndex = 0;
                     boundaryIndex < static_cast<int>(boundaryVertexIds.size());
                     ++boundaryIndex)
                {
                    if (boundaryVertexIds[boundaryIndex] == edgeSharedVertexId)
                    {
                        continue;
                    }

                    const int faceId = result.mesh.findFaceContainingVertices(
                        boundaryVertexIds[boundaryIndex],
                        edgeSharedVertexId
                    );
                    const CutHalfEdgePairResult connector =
                        CutPathMeshSplitter::createCutHalfEdges(
                            result.mesh,
                            boundaryVertexIds[boundaryIndex],
                            edgeSharedVertexId
                        );

                    if (faceId < 0 || !connector.success ||
                        !CutPathMeshSplitter::insertCutHalfEdgesIntoFace(
                            result.mesh,
                            faceId,
                            connector.firstHalfEdgeId,
                            connector.secondHalfEdgeId
                        ))
                    {
                        connected = false;
                        break;
                    }

                    boundaryToShared[boundaryIndex] =
                        connector.firstHalfEdgeId;
                    sharedToBoundary[boundaryIndex] =
                        connector.secondHalfEdgeId;
                }

                if (!connected)
                {
                    result.mesh = meshBeforeEdgeJunction;
                    edgeSharedVertexId = -1;
                }
                else
                {
                    result.embeddedVertexIds.insert(edgeSharedVertexId);
                    authoredNodeVertexIds.insert(edgeSharedVertexId);

                    for (int boundaryIndex = 0;
                         boundaryIndex < static_cast<int>(boundaryVertexIds.size());
                         ++boundaryIndex)
                    {
                        if (boundaryToShared[boundaryIndex] >= 0)
                        {
                            result.embeddedHalfEdgeIds.insert(
                                boundaryToShared[boundaryIndex]
                            );
                            result.embeddedHalfEdgeIds.insert(
                                sharedToBoundary[boundaryIndex]
                            );
                        }
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
                        point.meshVertexId = edgeSharedVertexId;
                        point.position = sharedPosition;

                        if (endpoint.endpoint == CurveEndpoint::Start)
                        {
                            chain.vertexIds.insert(
                                chain.vertexIds.begin(),
                                edgeSharedVertexId
                            );
                            chain.points.insert(chain.points.begin(), point);

                            if (sharedToBoundary[boundaryIndex] >= 0)
                            {
                                chain.halfEdgeIds.insert(
                                    chain.halfEdgeIds.begin(),
                                    sharedToBoundary[boundaryIndex]
                                );
                            }
                        }
                        else
                        {
                            chain.vertexIds.push_back(edgeSharedVertexId);
                            chain.points.push_back(point);

                            if (boundaryToShared[boundaryIndex] >= 0)
                            {
                                chain.halfEdgeIds.push_back(
                                    boundaryToShared[boundaryIndex]
                                );
                            }
                        }
                    }

                    continue;
                }
            }

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
                    sharedPosition,
                    std::max(surfaceTolerance, 0.001)
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

                int faceId = result.mesh.findFaceContainingVertices(
                    boundaryVertexIds[boundaryIndex],
                    sharedVertexId
                );

                if (faceId < 0)
                {
                    /*
                        Earlier incident cuts can divide the original face
                        into wedges before the shared node is inserted. Add
                        the same physical node to the neighboring wedge;
                        its auxiliary edge is ordinary mesh topology and is
                        deliberately not marked as a Curvenet barrier.
                    */
                    for (int candidateFaceId = 0;
                         candidateFaceId <
                             static_cast<int>(result.mesh.faces.size());
                         ++candidateFaceId)
                    {
                        const Point3& boundaryPosition =
                            result.mesh.vertices[
                                boundaryVertexIds[boundaryIndex]
                            ].position;
                        const Point3 faceProbe{
                            boundaryPosition.x +
                                (sharedPosition.x - boundaryPosition.x) *
                                    0.01,
                            boundaryPosition.y +
                                (sharedPosition.y - boundaryPosition.y) *
                                    0.01,
                            boundaryPosition.z +
                                (sharedPosition.z - boundaryPosition.z) *
                                    0.01
                        };

                        if (result.mesh.findOutgoingHalfEdgeInFace(
                                candidateFaceId,
                                boundaryVertexIds[boundaryIndex]
                            ) < 0 ||
                            !faceContainsPoint(
                                result.mesh,
                                candidateFaceId,
                                faceProbe,
                                std::max(surfaceTolerance, 0.001)
                            ))
                        {
                            continue;
                        }

                        const std::vector<int> candidateHalfEdges =
                            result.mesh.traverseFace(candidateFaceId);
                        int auxiliaryVertexId = -1;

                        for (int halfEdgeId : candidateHalfEdges)
                        {
                            const int vertexId =
                                result.mesh.halfEdges[halfEdgeId].startVertex;

                            if (vertexId ==
                                    boundaryVertexIds[boundaryIndex] ||
                                vertexId == sharedVertexId ||
                                std::find(
                                    boundaryVertexIds.begin(),
                                    boundaryVertexIds.end(),
                                    vertexId
                                ) != boundaryVertexIds.end())
                            {
                                continue;
                            }

                            auxiliaryVertexId = vertexId;
                            break;
                        }

                        if (auxiliaryVertexId < 0)
                        {
                            for (int halfEdgeId : candidateHalfEdges)
                            {
                                const int vertexId =
                                    result.mesh.halfEdges[halfEdgeId]
                                        .startVertex;

                                if (vertexId !=
                                        boundaryVertexIds[boundaryIndex] &&
                                    vertexId != sharedVertexId)
                                {
                                    auxiliaryVertexId = vertexId;
                                    break;
                                }
                            }
                        }

                        if (auxiliaryVertexId < 0)
                        {
                            continue;
                        }

                        const InteriorFaceSplitResult wedgeSplit =
                            result.mesh.splitFaceWithInteriorVertex(
                                candidateFaceId,
                                sharedVertexId,
                                {
                                    boundaryVertexIds[boundaryIndex],
                                    auxiliaryVertexId
                                }
                            );

                        if (!wedgeSplit.success)
                        {
                            continue;
                        }

                        boundaryToSharedHalfEdgeIds[boundaryIndex] =
                            wedgeSplit.boundaryToInteriorHalfEdgeIds[0];
                        sharedToBoundaryHalfEdgeIds[boundaryIndex] =
                            wedgeSplit.interiorToBoundaryHalfEdgeIds[0];
                        faceId = candidateFaceId;
                        break;
                    }

                    if (boundaryToSharedHalfEdgeIds[boundaryIndex] < 0)
                    {
                        int directionalFaceId = -1;
                        double bestDirectionalScore =
                            -std::numeric_limits<double>::infinity();
                        const Point3& boundaryPosition =
                            result.mesh.vertices[
                                boundaryVertexIds[boundaryIndex]
                            ].position;
                        const Point3 toShared = GeometryUtils::subtract(
                            sharedPosition,
                            boundaryPosition
                        );

                        for (int candidateFaceId = 0;
                             candidateFaceId <
                                 static_cast<int>(result.mesh.faces.size());
                             ++candidateFaceId)
                        {
                            if (result.mesh.findOutgoingHalfEdgeInFace(
                                    candidateFaceId,
                                    boundaryVertexIds[boundaryIndex]
                                ) < 0)
                            {
                                continue;
                            }

                            const std::vector<int> candidateHalfEdges =
                                result.mesh.traverseFace(candidateFaceId);
                            Point3 centroid;

                            for (int halfEdgeId : candidateHalfEdges)
                            {
                                const Point3& vertexPosition =
                                    result.mesh.vertices[
                                        result.mesh.halfEdges[halfEdgeId]
                                            .startVertex
                                    ].position;
                                centroid.x += vertexPosition.x;
                                centroid.y += vertexPosition.y;
                                centroid.z += vertexPosition.z;
                            }

                            if (candidateHalfEdges.empty())
                            {
                                continue;
                            }

                            const double inverseCount =
                                1.0 / candidateHalfEdges.size();
                            centroid.x *= inverseCount;
                            centroid.y *= inverseCount;
                            centroid.z *= inverseCount;
                            const Point3 toCentroid =
                                GeometryUtils::subtract(
                                    centroid,
                                    boundaryPosition
                                );
                            const double centroidLength =
                                GeometryUtils::length(toCentroid);

                            if (centroidLength <= 0.0000001)
                            {
                                continue;
                            }

                            const double score = GeometryUtils::dot(
                                toShared,
                                toCentroid
                            ) / centroidLength;

                            if (score > bestDirectionalScore)
                            {
                                bestDirectionalScore = score;
                                directionalFaceId = candidateFaceId;
                            }
                        }

                        if (directionalFaceId >= 0 &&
                            bestDirectionalScore > 0.0)
                        {
                            const std::vector<int> candidateHalfEdges =
                                result.mesh.traverseFace(directionalFaceId);
                            int auxiliaryVertexId = -1;

                            for (int halfEdgeId : candidateHalfEdges)
                            {
                                const int vertexId =
                                    result.mesh.halfEdges[halfEdgeId]
                                        .startVertex;

                                if (vertexId !=
                                        boundaryVertexIds[boundaryIndex] &&
                                    vertexId != sharedVertexId)
                                {
                                    auxiliaryVertexId = vertexId;
                                    break;
                                }
                            }

                            if (auxiliaryVertexId >= 0)
                            {
                                const InteriorFaceSplitResult wedgeSplit =
                                    result.mesh.splitFaceWithInteriorVertex(
                                        directionalFaceId,
                                        sharedVertexId,
                                        {
                                            boundaryVertexIds[boundaryIndex],
                                            auxiliaryVertexId
                                        }
                                    );

                                if (wedgeSplit.success)
                                {
                                    boundaryToSharedHalfEdgeIds[
                                        boundaryIndex
                                    ] = wedgeSplit
                                        .boundaryToInteriorHalfEdgeIds[0];
                                    sharedToBoundaryHalfEdgeIds[
                                        boundaryIndex
                                    ] = wedgeSplit
                                        .interiorToBoundaryHalfEdgeIds[0];
                                    faceId = directionalFaceId;
                                }
                            }
                        }
                    }

                    if (boundaryToSharedHalfEdgeIds[boundaryIndex] >= 0)
                    {
                        continue;
                    }
                }

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

    for (const ProfileCutInput& profileInput : profileInputs)
    {
        result.sampledSegmentsByCurveId[profileInput.curveId] =
            profileInput.sampledSegments;
    }

    /*
        Detect every authored curve on the same neutral mesh. Later CutPaths
        mutate the half-edge topology, but that must not change how remaining
        curves are interpreted on the original surface.
    */
    std::unordered_map<int, std::vector<CutCrossing>>
        neutralCrossingsByCurveId;
    std::unordered_map<int, std::vector<int>>
        neutralFaceIntervalsByCurveId;
    std::vector<std::vector<int>> neutralFaceIdsByVertex(
        inputMesh.vertices.size()
    );

    for (const HalfEdge& halfEdge : inputMesh.halfEdges)
    {
        if (halfEdge.startVertex >= 0 &&
            halfEdge.startVertex < static_cast<int>(
                neutralFaceIdsByVertex.size()
            ) &&
            halfEdge.face >= 0)
        {
            neutralFaceIdsByVertex[halfEdge.startVertex].push_back(
                halfEdge.face
            );
        }
    }

    for (std::vector<int>& faceIds : neutralFaceIdsByVertex)
    {
        std::sort(faceIds.begin(), faceIds.end());
        faceIds.erase(
            std::unique(faceIds.begin(), faceIds.end()),
            faceIds.end()
        );
    }

    for (const ProfileCutInput& profileInput : profileInputs)
    {
        SurfaceTrackedPath surfacePath =
            buildSurfaceTrackedPath(
                profileInput.curveId,
                profileInput.sampledSegments,
                inputMesh,
                neutralFaceIdsByVertex,
                profileInput.closed,
                crossingTolerance,
                duplicateTolerance
            );
        std::vector<CutCrossing> crossings = surfacePath.crossings;

        CutPath trackedPath;
        trackedPath.curveId = profileInput.curveId;
        trackedPath.closed = profileInput.closed;
        trackedPath.crossings = crossings;
        trackedPath.faceIntervalIds = surfacePath.faceIntervalIds;

        const int minimumCrossingCount =
            profileInput.closed ? 3 : 1;
        const int expectedTrackedIntervalCount =
            profileInput.closed
                ? static_cast<int>(crossings.size())
                : static_cast<int>(crossings.size()) - 1;
        const bool trackedPathIsComplete =
            static_cast<int>(crossings.size()) >= minimumCrossingCount &&
            surfacePath.reachedEndFace &&
            static_cast<int>(trackedPath.faceIntervalIds.size()) ==
                expectedTrackedIntervalCount &&
            std::all_of(
                trackedPath.faceIntervalIds.begin(),
                trackedPath.faceIntervalIds.end(),
                [](int faceId)
                {
                    return faceId >= 0;
                }
            );

        if (!trackedPathIsComplete)
        {
            SurfaceTrackingFailure failure;
            failure.curveId = profileInput.curveId;
            failure.crossingCount =
                static_cast<int>(crossings.size());
            failure.intervalCount = static_cast<int>(
                trackedPath.faceIntervalIds.size()
            );
            failure.invalidIntervalCount = static_cast<int>(
                std::count_if(
                    trackedPath.faceIntervalIds.begin(),
                    trackedPath.faceIntervalIds.end(),
                    [](int faceId)
                    {
                        return faceId < 0;
                    }
                )
            );
            result.surfaceTrackingFailures.push_back(failure);

            crossings = CurveMeshIntersector::findAllCrossings(
                profileInput.curveId,
                profileInput.sampledSegments,
                inputMesh,
                crossingTolerance,
                duplicateTolerance
            );
            trackedPath.crossings = crossings;
            trackedPath.faceIntervalIds =
                CurveMeshIntersector::deriveFaceIntervals(
                    trackedPath,
                    inputMesh
                );
        }
        else
        {
            ++result.surfaceTrackedCurveCount;
        }

        neutralCrossingsByCurveId[profileInput.curveId] =
            std::move(crossings);
        neutralFaceIntervalsByCurveId[profileInput.curveId] =
            std::move(trackedPath.faceIntervalIds);
    }

    for (const ProfileCutInput& profileInput :
         profileInputs)
    {
        std::vector<CutCrossing> crossings =
            neutralCrossingsByCurveId.at(profileInput.curveId);

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

        const std::vector<int>& neutralIntervals =
            neutralFaceIntervalsByCurveId.at(profileInput.curveId);
        const int expectedNeutralIntervalCount =
            cutPath.closed
                ? static_cast<int>(cutPath.crossings.size())
                : std::max(
                      0,
                      static_cast<int>(cutPath.crossings.size()) - 1
                  );

        if (static_cast<int>(neutralIntervals.size()) ==
            expectedNeutralIntervalCount)
        {
            cutPath.faceIntervalIds = neutralIntervals;
        }
        else
        {
            cutPath.faceIntervalIds =
                CurveMeshIntersector::deriveFaceIntervals(
                    cutPath,
                    result.mesh
                );
        }

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
            logicalEndpointNodes,
            std::max(
                0.00001,
                duplicateTolerance * 2.0
            )
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
