#pragma once

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
