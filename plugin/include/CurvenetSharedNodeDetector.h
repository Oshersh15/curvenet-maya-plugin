#pragma once

#include <optional>

#include "CurvenetCutResult.h"
#include "CutVertex.h"

class CurvenetSharedNodeDetector
{
public:
    static std::optional<int> findSharedMeshVertex(
        const CutVertex& endpoint,
        const CurvenetCutResult& curvenetResult,
        double positionTolerance
    );
};
