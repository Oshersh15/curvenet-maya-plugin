/* Detects endpoint-to-curve relationships in sampled authored profiles. */

#include "CurveConnectionDetector.h"

#include "ProfileCurveSampler.h"

namespace
{
    bool connectionAlreadyExists(
        const std::vector<DetectedCurveConnection>& connections,
        const DetectedCurveConnection& candidate,
        double tolerance
    )
    {
        for (const DetectedCurveConnection& existing : connections)
        {
            const bool sameCurvePair =
                (
                    existing.endpointCurveId ==
                        candidate.endpointCurveId &&
                    existing.targetCurveId ==
                        candidate.targetCurveId
                ) ||
                (
                    existing.endpointCurveId ==
                        candidate.targetCurveId &&
                    existing.targetCurveId ==
                        candidate.endpointCurveId
                );

            if (!sameCurvePair)
            {
                continue;
            }

            const double positionDistance =
                GeometryUtils::pointToPointDistance(
                    existing.position,
                    candidate.position
                );

            if (positionDistance <= tolerance)
            {
                return true;
            }
        }

        return false;
    }
}

std::vector<DetectedCurveConnection>
CurveConnectionDetector::detect(
    const std::vector<std::vector<Point3>>& sampledCurves,
    double tolerance
)
{
    std::vector<DetectedCurveConnection> connections;

    for (int endpointCurveId = 0;
         endpointCurveId <
             static_cast<int>(sampledCurves.size());
         ++endpointCurveId)
    {
        const std::vector<Point3>& endpointCurve =
            sampledCurves[endpointCurveId];

        if (endpointCurve.empty())
        {
            continue;
        }

        struct EndpointInfo
        {
            CurveEndpoint endpoint;
            Point3 position;
        };

        const EndpointInfo endpoints[2] =
        {
            {
                CurveEndpoint::Start,
                endpointCurve.front()
            },
            {
                CurveEndpoint::End,
                endpointCurve.back()
            }
        };

        for (const EndpointInfo& endpointInfo : endpoints)
        {
            for (int targetCurveId = 0;
                 targetCurveId <
                     static_cast<int>(sampledCurves.size());
                 ++targetCurveId)
            {
                if (endpointCurveId == targetCurveId)
                {
                    continue;
                }

                const std::vector<PolylineSegment> targetSegments =
                    ProfileCurveSampler::buildPolylineSegments(
                        sampledCurves[targetCurveId]
                    );

                const ClosestCurveSegmentResult result =
                    GeometryUtils::findClosestPolylineSegment(
                        endpointInfo.position,
                        targetSegments
                    );

                if (!result.found ||
                    result.distance > tolerance)
                {
                    continue;
                }

                DetectedCurveConnection connection;

                connection.endpointCurveId =
                    endpointCurveId;

                connection.endpoint =
                    endpointInfo.endpoint;

                connection.targetCurveId =
                    targetCurveId;

                connection.targetSegmentId =
                    result.segmentId;

                connection.targetSegmentT =
                    result.segmentT;

                connection.position =
                    result.closestPoint;

                if (!connectionAlreadyExists(
                        connections,
                        connection,
                        tolerance
                    ))
                {
                    connections.push_back(connection);
                }
            }
        }
    }

    return connections;
}
