#pragma once

#include "CurvenetCutResult.h"

class CurvenetFaceRegionBuilder
{
public:
    static void build(
        CurvenetCutResult& cutResult
    );

    static void buildFullSurfacePartitions(
        CurvenetCutResult& cutResult
    );
};
