#pragma once

#include "CurvenetCutResult.h"
#include "ProfileCutInput.h"

struct ProfileCutInput;

class CurvenetEdgeBuilder
{
public:

    static void build(
        CurvenetCutResult& curvenetCutResult,
        const std::vector<ProfileCutInput>& profileInputs
    );
};
