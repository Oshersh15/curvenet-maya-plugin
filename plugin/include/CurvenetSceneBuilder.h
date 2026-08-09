#pragma once

#include "CurvenetCutResult.h"

#include <maya/MString.h>

class CurvenetSceneBuilder
{
public:

    static void build(
        const CurvenetCutResult& curvenetCutResult,
        const MString& ownerName,
        const MString& geometryTransformName
    );
};
