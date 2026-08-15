#pragma once

/* Declares creation of the optional coloured surface-region preview. */

#include "CurvenetCutResult.h"

#include <string>

class CurvenetRegionPreviewBuilder
{
public:
    static void build(
        const std::string& ownerName,
        const CurvenetCutResult& curvenetCutResult
    );
};
