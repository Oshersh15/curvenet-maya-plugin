#pragma once

#include "CutCrossing.h"

#include <vector>

struct CutPath
{
    int curveId = -1;

    std::vector<CutCrossing> crossings;
};
