#pragma once

#include <vector>

#include "CurvenetCutResult.h"
#include "CurvenetFace.h"

class CurvenetFaceBuilder
{
public:
    static std::vector<CurvenetFace> build(
        const CurvenetCutResult& cutResult
    );
};
