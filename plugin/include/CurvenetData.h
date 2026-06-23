#pragma once

#include <maya/MObject.h>
#include <maya/MPoint.h>

#include <vector>

struct ProfileCurveData
{
    int id;
    MObject curveObject;
    std::vector<MPoint> restCVPositions;
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

private:
    std::vector<ProfileCurveData> m_curves;
};
