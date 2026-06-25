#pragma once

#include <maya/MObject.h>
#include <maya/MPoint.h>

#include <vector>

enum class CurveEndpoint
{
    Start,
    End
};

struct CurveConnection
{
    int firstCurveId;
    CurveEndpoint firstEndpoint;

    int secondCurveId;
    CurveEndpoint secondEndpoint;

    MPoint position;
};

struct ProfileCurveData
{
    int id;
    MObject curveObject;
    std::vector<MPoint> restCVPositions;

    MPoint startPoint;
    MPoint endPoint;
};

class CurvenetData
{
public:
    void clear();

    void addCurve(
        const MObject& curveObject,
        const std::vector<MPoint>& cvPositions);

    int getCurveCount() const;

    const std::vector<ProfileCurveData>& getCurves() const;

    void detectConnections(double tolerance);

    const std::vector<CurveConnection>& getConnections() const;

    std::vector<int> getConnectedCurves(int curveId) const;

private:
    std::vector<ProfileCurveData> m_curves;
    std::vector<CurveConnection> m_connections;
};
