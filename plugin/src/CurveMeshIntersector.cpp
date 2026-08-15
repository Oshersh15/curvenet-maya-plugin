/* Finds ordered crossings between sampled profile segments and mesh edges. */

#include "CurveMeshIntersector.h"

#include "GeometryUtils.h"
#include <algorithm>

FirstCrossingResult CurveMeshIntersector::findFirstCrossing(
    int curveId,
    const std::vector<PolylineSegment>& curveSegments,
    const HalfEdgeMesh& mesh,
    double tolerance
)

{
    FirstCrossingResult result;

    for (int curveSegmentIndex = 0;
         curveSegmentIndex < static_cast<int>(curveSegments.size());
         ++curveSegmentIndex)
    {
        const PolylineSegment& curveSegment =
            curveSegments[curveSegmentIndex];

        for (int halfEdgeIndex = 0;
             halfEdgeIndex < static_cast<int>(mesh.halfEdges.size());
             ++halfEdgeIndex)
        {
            const HalfEdge& halfEdge =
                mesh.halfEdges[halfEdgeIndex];

            if (halfEdge.twin >= 0 &&
                halfEdgeIndex > halfEdge.twin)
            {
                continue;
            }

            if (halfEdge.startVertex < 0 ||
                halfEdge.startVertex >= static_cast<int>(mesh.vertices.size()) ||
                halfEdge.endVertex < 0 ||
                halfEdge.endVertex >= static_cast<int>(mesh.vertices.size()))
            {
                continue;
            }

            const Point3& meshEdgeStart =
                mesh.vertices[halfEdge.startVertex].position;

            const Point3& meshEdgeEnd =
                mesh.vertices[halfEdge.endVertex].position;

            SegmentDistanceResult distanceResult =
                GeometryUtils::segmentToSegmentDistance(
                    curveSegment.start,
                    curveSegment.end,
                    meshEdgeStart,
                    meshEdgeEnd
                );

            if (distanceResult.distance <= tolerance)
            {
                result.found = true;

                result.crossing.curveId = curveId;
                result.crossing.curveSegmentId = curveSegmentIndex;
                result.crossing.faceId = halfEdge.face;
                result.crossing.halfEdgeId = halfEdgeIndex;
                result.crossing.meshEdgeT =
                    distanceResult.secondSegmentT;
                result.crossing.position =
                    distanceResult.closestPointOnSecondSegment;

                result.distance = distanceResult.distance;

                return result;
            }
        }
    }

    return result;
}

std::vector<CutCrossing> CurveMeshIntersector::findAllCrossings(
    int curveId,
    const std::vector<PolylineSegment>& curveSegments,
    const HalfEdgeMesh& mesh,
    double crossingTolerance,
    double duplicateTolerance
)

{
    std::vector<CutCrossing> crossings;

    for (int curveSegmentIndex = 0;
         curveSegmentIndex < static_cast<int>(curveSegments.size());
         ++curveSegmentIndex)
    {
        const PolylineSegment& curveSegment =
            curveSegments[curveSegmentIndex];

        for (int halfEdgeIndex = 0;
             halfEdgeIndex < static_cast<int>(mesh.halfEdges.size());
             ++halfEdgeIndex)
        {
            const HalfEdge& halfEdge =
                mesh.halfEdges[halfEdgeIndex];

            if (halfEdge.twin >= 0 &&
                halfEdgeIndex > halfEdge.twin)
            {
                continue;
            }

            if (halfEdge.startVertex < 0 ||
                halfEdge.startVertex >= static_cast<int>(mesh.vertices.size()) ||
                halfEdge.endVertex < 0 ||
                halfEdge.endVertex >= static_cast<int>(mesh.vertices.size()))
            {
                continue;
            }

            const Point3& meshEdgeStart =
                mesh.vertices[halfEdge.startVertex].position;

            const Point3& meshEdgeEnd =
                mesh.vertices[halfEdge.endVertex].position;

            SegmentDistanceResult distanceResult =
                GeometryUtils::segmentToSegmentDistance(
                    curveSegment.start,
                    curveSegment.end,
                    meshEdgeStart,
                    meshEdgeEnd
                );

            if (distanceResult.distance > crossingTolerance)
            {
                continue;
            }

            const bool nearlyParallel =
                GeometryUtils::areSegmentsNearlyParallel(
                    curveSegment.start,
                    curveSegment.end,
                    meshEdgeStart,
                    meshEdgeEnd,
                    0.001
                );

            if (nearlyParallel)
            {
                continue;
            }

            CutCrossing candidate;

            candidate.curveId = curveId;
            candidate.curveSegmentId = curveSegmentIndex;
            candidate.curveSegmentT =
                distanceResult.firstSegmentT;
            candidate.faceId = halfEdge.face;
            candidate.halfEdgeId = halfEdgeIndex;
            candidate.meshEdgeT =
                distanceResult.secondSegmentT;
            candidate.position =
                distanceResult.closestPointOnSecondSegment;

            bool duplicateFound = false;

            for (const CutCrossing& existingCrossing : crossings)
            {
                const double positionDistance =
                    GeometryUtils::pointToPointDistance(
                        candidate.position,
                        existingCrossing.position
                    );

                if (positionDistance <= duplicateTolerance)
                {
                    duplicateFound = true;
                    break;
                }
            }

            if (!duplicateFound)
            {
                crossings.push_back(candidate);
            }
        }
    }
    std::sort(
        crossings.begin(),
        crossings.end(),
        [](const CutCrossing& first,
           const CutCrossing& second)
        {
            if (first.curveSegmentId !=
                second.curveSegmentId)
            {
                return first.curveSegmentId <
                       second.curveSegmentId;
            }

            return first.curveSegmentT <
                   second.curveSegmentT;
        }
    );

    return crossings;
}

std::vector<CutVertex>
CurveMeshIntersector::buildCutVertices(
    const std::vector<CutCrossing>& crossings,
    double duplicateTolerance
)
{
    std::vector<CutVertex> cutVertices;

    for (const CutCrossing& crossing : crossings)
    {
        bool duplicateFound = false;

        for (const CutVertex& existingVertex : cutVertices)
        {
            const double positionDistance =
                GeometryUtils::pointToPointDistance(
                    crossing.position,
                    existingVertex.position
                );

            if (positionDistance <= duplicateTolerance)
            {
                duplicateFound = true;
                break;
            }
        }

        if (duplicateFound)
        {
            continue;
        }

        CutVertex cutVertex;

        cutVertex.position =
            crossing.position;

        cutVertex.sourceHalfEdgeId =
            crossing.halfEdgeId;

        cutVertex.sourceEdgeT =
            crossing.meshEdgeT;

        cutVertex.curveId =
            crossing.curveId;

        cutVertex.curveSegmentId =
            crossing.curveSegmentId;

        cutVertex.curveSegmentT =
            crossing.curveSegmentT;

        cutVertex.cutPathOrder =
            static_cast<int>(cutVertices.size());

        cutVertices.push_back(
            cutVertex
        );
    }

    return cutVertices;
}

std::vector<int> CurveMeshIntersector::deriveFaceIntervals(
    const CutPath& cutPath,
    const HalfEdgeMesh& mesh
)
{
    std::vector<int> intervalFaceIds;

    const auto collectCrossingFaces =
        [&mesh](
            const CutCrossing& crossing
        )
    {
        std::vector<int> adjacentFaces;

        const auto addFace =
            [&adjacentFaces](int faceId)
        {
            if (faceId < 0)
            {
                return;
            }

            if (std::find(
                    adjacentFaces.begin(),
                    adjacentFaces.end(),
                    faceId
                ) == adjacentFaces.end())
            {
                adjacentFaces.push_back(faceId);
            }
        };

        if (crossing.halfEdgeId < 0 ||
            crossing.halfEdgeId >=
                static_cast<int>(
                    mesh.halfEdges.size()
                ))
        {
            return adjacentFaces;
        }

        const HalfEdge& halfEdge =
            mesh.halfEdges[
                crossing.halfEdgeId
            ];

        addFace(halfEdge.face);

        if (halfEdge.twin >= 0 &&
            halfEdge.twin <
                static_cast<int>(
                    mesh.halfEdges.size()
                ))
        {
            addFace(
                mesh.halfEdges[
                    halfEdge.twin
                ].face
            );
        }

        /*
            A crossing snapped exactly to a mesh vertex may
            enter and leave through any face incident to that
            vertex, not only the two faces beside its original
            source half-edge.
        */
        const int endpointVertexIds[2] = {
            halfEdge.startVertex,
            halfEdge.endVertex
        };

        for (int endpointVertexId :
             endpointVertexIds)
        {
            if (endpointVertexId < 0 ||
                endpointVertexId >=
                    static_cast<int>(
                        mesh.vertices.size()
                    ))
            {
                continue;
            }

            const double endpointDistance =
                GeometryUtils::pointToPointDistance(
                    crossing.position,
                    mesh.vertices[
                        endpointVertexId
                    ].position
                );

            if (endpointDistance > 0.0000001)
            {
                continue;
            }

            for (const HalfEdge& incidentHalfEdge :
                 mesh.halfEdges)
            {
                if (incidentHalfEdge.startVertex ==
                        endpointVertexId ||
                    incidentHalfEdge.endVertex ==
                        endpointVertexId)
                {
                    addFace(
                        incidentHalfEdge.face
                    );
                }
            }
        }

        return adjacentFaces;
    };

    if (cutPath.crossings.size() < 2)
    {
        return intervalFaceIds;
    }

    for (int crossingIndex = 0;
         crossingIndex <
             static_cast<int>(cutPath.crossings.size()) - 1;
         ++crossingIndex)
    {
        const CutCrossing& firstCrossing =
            cutPath.crossings[crossingIndex];

        const CutCrossing& secondCrossing =
            cutPath.crossings[crossingIndex + 1];

        if (firstCrossing.halfEdgeId < 0 ||
            firstCrossing.halfEdgeId >=
                static_cast<int>(mesh.halfEdges.size()) ||
            secondCrossing.halfEdgeId < 0 ||
            secondCrossing.halfEdgeId >=
                static_cast<int>(mesh.halfEdges.size()))
        {
            intervalFaceIds.push_back(-1);
            continue;
        }

        const std::vector<int> firstAdjacentFaces =
            collectCrossingFaces(
                firstCrossing
            );

        const std::vector<int> secondAdjacentFaces =
            collectCrossingFaces(
                secondCrossing
            );

        int sharedFaceId = -1;

        for (int firstFaceId : firstAdjacentFaces)
        {
            for (int secondFaceId : secondAdjacentFaces)
            {
                if (firstFaceId == secondFaceId)
                {
                    sharedFaceId = firstFaceId;
                    break;
                }
            }

            if (sharedFaceId >= 0)
            {
                break;
            }
        }

        intervalFaceIds.push_back(sharedFaceId);
    }

    if (cutPath.closed)
    {
        const CutCrossing& firstCrossing =
            cutPath.crossings.back();

        const CutCrossing& secondCrossing =
            cutPath.crossings.front();

        if (firstCrossing.halfEdgeId < 0 ||
            firstCrossing.halfEdgeId >=
                static_cast<int>(mesh.halfEdges.size()) ||
            secondCrossing.halfEdgeId < 0 ||
            secondCrossing.halfEdgeId >=
                static_cast<int>(mesh.halfEdges.size()))
        {
            intervalFaceIds.push_back(-1);
            return intervalFaceIds;
        }

        const std::vector<int> firstAdjacentFaces =
            collectCrossingFaces(
                firstCrossing
            );

        const std::vector<int> secondAdjacentFaces =
            collectCrossingFaces(
                secondCrossing
            );

        int sharedFaceId = -1;

        for (int firstFaceId : firstAdjacentFaces)
        {
            for (int secondFaceId : secondAdjacentFaces)
            {
                if (firstFaceId == secondFaceId)
                {
                    sharedFaceId =
                        firstFaceId;

                    break;
                }
            }

            if (sharedFaceId >= 0)
            {
                break;
            }
        }

        intervalFaceIds.push_back(
            sharedFaceId
        );
    }

    return intervalFaceIds;
}

std::vector<int> CurveMeshIntersector::collectUniqueFaces(
    const std::vector<int>& faceIntervals
)
{
    std::vector<int> collectedFaceIds;

    for (int faceId : faceIntervals)
    {
        if (faceId == -1)
        {
            continue;
        }

        bool alreadyCollected = false;

        for (int collectedFaceId : collectedFaceIds)
        {
            if (collectedFaceId == faceId)
            {
                alreadyCollected = true;
                break;
            }
        }

        if (!alreadyCollected)
        {
            collectedFaceIds.push_back(faceId);
        }
    }

    return collectedFaceIds;
}
