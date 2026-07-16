#pragma once

#include "GeometryUtils.h"

#include <vector>

enum class CurveEndpoint
{
    Start,
    End
};

struct DetectedCurveConnection
{
    int endpointCurveId = -1;
    CurveEndpoint endpoint = CurveEndpoint::Start;

    int targetCurveId = -1;
    int targetSegmentId = -1;
    double targetSegmentT = 0.0;

    Point3 position;
};

class CurveConnectionDetector
{
public:
    static std::vector<DetectedCurveConnection> detect(
        const std::vector<std::vector<Point3>>& sampledCurves,
        double tolerance
    );
};
