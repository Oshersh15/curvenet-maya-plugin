#pragma once

#include <maya/MObject.h>
#include <maya/MPoint.h>
#include "HalfEdge.h"

#include <vector>

enum class CurveEndpoint
{
    Start,
    End
};

struct CurveConnection
{
    int endpointCurveId = -1;
    CurveEndpoint endpoint =
        CurveEndpoint::Start;

    int targetCurveId = -1;
    int targetSegmentId = -1;
    double targetSegmentT = 0.0;

    MPoint position;
};

struct ProfileCurveData
{
    int id = -1;
    MObject curveObject;
    std::vector<MPoint> restCVPositions;
    std::vector<Point3> sampledPoints;

    MPoint startPoint;
    MPoint endPoint;
};

class CurvenetData
{
public:
    void clear();

    void addCurve(
        const MObject& curveObject,
        const std::vector<MPoint>& cvPositions,
        const std::vector<Point3>& sampledPoints
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
