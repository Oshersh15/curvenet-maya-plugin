#include "CurvenetData.h"

void CurvenetData::clear()
{
    m_curves.clear();
    m_connections.clear();
}

void CurvenetData::addCurve(
    const MObject& curveObject,
    const std::vector<MPoint>& cvPositions)
{
    ProfileCurveData curve;

    curve.id = static_cast<int>(m_curves.size());
    curve.curveObject = curveObject;
    curve.restCVPositions = cvPositions;

    if (!cvPositions.empty())
    {
        curve.startPoint = cvPositions.front();
        curve.endPoint = cvPositions.back();
    }

    m_curves.push_back(curve);
}

void CurvenetData::detectConnections(double tolerance)
{
    m_connections.clear();

    for (size_t i = 0; i < m_curves.size(); ++i)
    {
        for (size_t j = i + 1; j < m_curves.size(); ++j)
        {
            const ProfileCurveData& firstCurve = m_curves[i];
            const ProfileCurveData& secondCurve = m_curves[j];

            struct EndpointInfo
            {
                CurveEndpoint endpoint;
                MPoint position;
            };

            EndpointInfo firstEndpoints[2] = {
                { CurveEndpoint::Start, firstCurve.startPoint },
                { CurveEndpoint::End, firstCurve.endPoint }
            };

            EndpointInfo secondEndpoints[2] = {
                { CurveEndpoint::Start, secondCurve.startPoint },
                { CurveEndpoint::End, secondCurve.endPoint }
            };

            for (const EndpointInfo& firstEndpoint : firstEndpoints)
            {
                for (const EndpointInfo& secondEndpoint : secondEndpoints)
                {
                    double distance = firstEndpoint.position.distanceTo(secondEndpoint.position);

                    if (distance <= tolerance)
                    {
                        CurveConnection connection;
                        connection.firstCurveId = firstCurve.id;
                        connection.firstEndpoint = firstEndpoint.endpoint;
                        connection.secondCurveId = secondCurve.id;
                        connection.secondEndpoint = secondEndpoint.endpoint;
                        connection.position = firstEndpoint.position;

                        m_connections.push_back(connection);
                    }
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

std::vector<int> CurvenetData::getConnectedCurves(int curveId) const
{
    std::vector<int> connectedCurves;

    for (const auto& connection : m_connections)
    {
        if (connection.firstCurveId == curveId)
        {
            connectedCurves.push_back(connection.secondCurveId);
        }

        if (connection.secondCurveId == curveId)
        {
            connectedCurves.push_back(connection.firstCurveId);
        }
    }

    return connectedCurves;
}
