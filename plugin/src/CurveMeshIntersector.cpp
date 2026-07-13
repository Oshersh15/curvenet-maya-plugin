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

            CutCrossing candidate;

            candidate.curveId = curveId;
            candidate.curveSegmentId = curveSegmentIndex;
            candidate.curveSegmentT =
                distanceResult.firstSegmentT;
            candidate.faceId = halfEdge.face;
            candidate.halfEdgeId = halfEdgeIndex;
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

std::vector<int> CurveMeshIntersector::deriveFaceIntervals(
    const CutPath& cutPath,
    const HalfEdgeMesh& mesh
)
{
    std::vector<int> intervalFaceIds;

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

        const HalfEdge& firstHalfEdge =
            mesh.halfEdges[firstCrossing.halfEdgeId];

        const HalfEdge& secondHalfEdge =
            mesh.halfEdges[secondCrossing.halfEdgeId];

        std::vector<int> firstAdjacentFaces;
        std::vector<int> secondAdjacentFaces;

        if (firstHalfEdge.face >= 0)
        {
            firstAdjacentFaces.push_back(
                firstHalfEdge.face
            );
        }

        if (firstHalfEdge.twin >= 0 &&
            firstHalfEdge.twin <
                static_cast<int>(mesh.halfEdges.size()))
        {
            const int twinFace =
                mesh.halfEdges[firstHalfEdge.twin].face;

            if (twinFace >= 0)
            {
                firstAdjacentFaces.push_back(twinFace);
            }
        }

        if (secondHalfEdge.face >= 0)
        {
            secondAdjacentFaces.push_back(
                secondHalfEdge.face
            );
        }

        if (secondHalfEdge.twin >= 0 &&
            secondHalfEdge.twin <
                static_cast<int>(mesh.halfEdges.size()))
        {
            const int twinFace =
                mesh.halfEdges[secondHalfEdge.twin].face;

            if (twinFace >= 0)
            {
                secondAdjacentFaces.push_back(twinFace);
            }
        }

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
