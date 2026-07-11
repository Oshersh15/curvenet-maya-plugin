#include "CurveMeshIntersector.h"

#include "GeometryUtils.h"

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
    return crossings;
}
