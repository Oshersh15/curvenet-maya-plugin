/* Stores the sampled curves and explicit connections read from Maya inputs. */

#include "CurvenetData.h"
#include "GeometryUtils.h"
#include "ProfileCurveSampler.h"

#include <maya/MGlobal.h>
#include <maya/MString.h>

void CurvenetData::clear()
{
    m_curves.clear();
    m_connections.clear();
}

void CurvenetData::addCurve(
    const MObject& curveObject,
    const std::vector<MPoint>& cvPositions,
    const std::vector<Point3>& sampledPoints,
    bool closed
)
{
    ProfileCurveData curve;
    curve.closed = closed;

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
    std::vector<std::vector<Point3>> sampledCurves;

    sampledCurves.reserve(m_curves.size());

    for (const ProfileCurveData& curve : m_curves)
    {
        sampledCurves.push_back(
            curve.sampledPoints
        );
    }

    const std::vector<DetectedCurveConnection>
        detectedConnections =
            CurveConnectionDetector::detect(
                sampledCurves,
                tolerance
            );

    m_connections.clear();
    m_connections.reserve(
        detectedConnections.size()
    );

    for (const DetectedCurveConnection& detected :
         detectedConnections)
    {
        CurveConnection connection;

        connection.endpointCurveId =
            detected.endpointCurveId;

        connection.endpoint =
            detected.endpoint;

        connection.targetCurveId =
            detected.targetCurveId;

        connection.targetSegmentId =
            detected.targetSegmentId;

        connection.targetSegmentT =
            detected.targetSegmentT;

        connection.position =
            MPoint(
                detected.position.x,
                detected.position.y,
                detected.position.z
            );

        m_connections.push_back(connection);
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
