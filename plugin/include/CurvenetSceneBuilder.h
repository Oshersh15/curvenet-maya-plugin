#pragma once

/* Declares creation of Curvenet controls and viewport curve geometry. */

#include "CurvenetCutResult.h"

#include <maya/MString.h>

class CurvenetSceneBuilder
{
public:

    static void build(
        const CurvenetCutResult& curvenetCutResult,
        const MString& ownerName,
        const MString& geometryTransformName,
        bool showGeneratedCurvenet
    );
};
