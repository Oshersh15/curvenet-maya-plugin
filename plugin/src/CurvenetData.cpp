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
    const std::vector<Point3>& sampledPoints,
    bool closed
)
{
    ProfileCurveData curve;
    curve.closed = closed;

    curve.sampledPoints = sampledPoints;

    if (!sampledPoints.empty())
    {
        curve.startPoint = sampledPoints.front();
        curve.endPoint = sampledPoints.back();
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

        connection.position = detected.position;

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
