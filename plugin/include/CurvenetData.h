#pragma once

#include "HalfEdge.h"
#include "CurveConnectionDetector.h"

#include <vector>

struct CurveConnection
{
    int endpointCurveId = -1;
    CurveEndpoint endpoint =
        CurveEndpoint::Start;

    int targetCurveId = -1;
    int targetSegmentId = -1;
    double targetSegmentT = 0.0;

    Point3 position;
};

struct ProfileCurveData
{
    int id = -1;
    std::vector<Point3> sampledPoints;

    Point3 startPoint;
    Point3 endPoint;

    bool closed = false;
};

class CurvenetData
{
public:
    void clear();

    void addCurve(
        const std::vector<Point3>& sampledPoints,
        bool closed
    );

    int getCurveCount() const;

    const std::vector<ProfileCurveData>& getCurves() const;

    void detectConnections(double tolerance);

    const std::vector<CurveConnection>& getConnections() const;

    std::vector<int> getConnectedCurves(int curveId) const;

private:
    std::vector<ProfileCurveData> m_curves;
    std::vector<CurveConnection> m_connections;
};
