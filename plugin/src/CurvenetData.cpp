#include "CurvenetData.h"

void CurvenetData::clear()
{
    m_curves.clear();
}

void CurvenetData::addCurve(
    const MObject& curveObject,
    const std::vector<MPoint>& cvPositions)
{
    ProfileCurveData curve;

    curve.id = static_cast<int>(m_curves.size());
    curve.curveObject = curveObject;
    curve.restCVPositions = cvPositions;

    m_curves.push_back(curve);
}

int CurvenetData::getCurveCount() const
{
    return static_cast<int>(m_curves.size());
}

const std::vector<ProfileCurveData>&
CurvenetData::getCurves() const
{
    return m_curves;
}
