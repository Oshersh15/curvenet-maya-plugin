#include "CurvenetData.h"
#include "GeometryUtils.h"
#include "ProfileCurveSampler.h"

namespace
{
    ClosestCurveSegmentResult findClosestPointOnCurve(
        const MPoint& endpointPosition,
        const ProfileCurveData& targetCurve
    )
    {
        const Point3 endpoint{
            endpointPosition.x,
            endpointPosition.y,
            endpointPosition.z
        };

        const std::vector<PolylineSegment> segments =
            ProfileCurveSampler::buildPolylineSegments(
                targetCurve.sampledPoints
            );

        return GeometryUtils::findClosestPolylineSegment(
            endpoint,
            segments
        );
    }

    bool connectionAlreadyExists(
        const std::vector<CurveConnection>& connections,
        const CurveConnection& candidate,
        double tolerance
    )
    {
        for (const CurveConnection& existing : connections)
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

            if (existing.position.distanceTo(
                    candidate.position
                ) <= tolerance)
            {
                return true;
            }
        }

        return false;
    }
}

void CurvenetData::clear()
{
    m_curves.clear();
    m_connections.clear();
}

void CurvenetData::addCurve(
    const MObject& curveObject,
    const std::vector<MPoint>& cvPositions,
    const std::vector<Point3>& sampledPoints
)
{
    ProfileCurveData curve;
    curve.sampledPoints = sampledPoints;

    curve.curveObject = curveObject;
    curve.restCVPositions = cvPositions;
    curve.sampledPoints = sampledPoints;

    if (!sampledPoints.empty())
    {
        curve.startPoint = MPoint(
            sampledPoints.front().x,
            sampledPoints.front().y,
            sampledPoints.front().z
        );

        curve.endPoint = MPoint(
            sampledPoints.back().x,
            sampledPoints.back().y,
            sampledPoints.back().z
        );
    }
    else if (!cvPositions.empty())
    {
        curve.startPoint = cvPositions.front();
        curve.endPoint = cvPositions.back();
    }

    m_curves.push_back(curve);
}

void CurvenetData::detectConnections(
    double tolerance
)
{
    m_connections.clear();

    for (size_t i = 0;
         i < m_curves.size();
         ++i)
    {
        const ProfileCurveData& endpointCurve =
            m_curves[i];

        struct EndpointInfo
        {
            CurveEndpoint endpoint;
            MPoint position;
        };

        EndpointInfo endpoints[2] =
        {
            {
                CurveEndpoint::Start,
                endpointCurve.startPoint
            },
            {
                CurveEndpoint::End,
                endpointCurve.endPoint
            }
        };

        for (const EndpointInfo& endpointInfo :
             endpoints)
        {
            for (size_t j = 0;
                 j < m_curves.size();
                 ++j)
            {
                if (i == j)
                {
                    continue;
                }

                const ProfileCurveData& targetCurve =
                    m_curves[j];

                const ClosestCurveSegmentResult result =
                    findClosestPointOnCurve(
                        endpointInfo.position,
                        targetCurve
                    );

                if (!result.found)
                {
                    continue;
                }

                if (result.distance > tolerance)
                {
                    continue;
                }

                CurveConnection connection;

                connection.endpointCurveId =
                    endpointCurve.id;

                connection.endpoint =
                    endpointInfo.endpoint;

                connection.targetCurveId =
                    targetCurve.id;

                connection.targetSegmentId =
                    result.segmentId;

                connection.targetSegmentT =
                    result.segmentT;

                connection.position =
                    MPoint(
                        result.closestPoint.x,
                        result.closestPoint.y,
                        result.closestPoint.z
                    );

                if (!connectionAlreadyExists(
                        m_connections,
                        connection,
                        tolerance
                    ))
                {
                    m_connections.push_back(
                        connection
                    );
                }
            }
        }
    }
}

const std::vector<CurveConnection>& CurvenetData::getConnections() const
{
    return m_connections;
}

int CurvenetData::getCurveCount() const
{
    return static_cast<int>(m_curves.size());
}

const std::vector<ProfileCurveData>& CurvenetData::getCurves() const
{
    return m_curves;
}

std::vector<int> CurvenetData::getConnectedCurves(
    int curveId
) const
{
    std::vector<int> connectedCurves;

    for (const CurveConnection& connection :
         m_connections)
    {
        int connectedCurveId = -1;

        if (connection.endpointCurveId == curveId)
        {
            connectedCurveId =
                connection.targetCurveId;
        }
        else if (connection.targetCurveId == curveId)
        {
            connectedCurveId =
                connection.endpointCurveId;
        }

        if (connectedCurveId < 0)
        {
            continue;
        }

        bool alreadyStored = false;

        for (int existingCurveId : connectedCurves)
        {
            if (existingCurveId == connectedCurveId)
            {
                alreadyStored = true;
                break;
            }
        }

        if (!alreadyStored)
        {
            connectedCurves.push_back(
                connectedCurveId
            );
        }
    }

    return connectedCurves;
}
