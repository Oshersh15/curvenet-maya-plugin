#pragma once

#include "CurvenetCutResult.h"
#include <vector>

struct TransferredRegionTriangle
{
    int regionId = -1;
    Point3 first;
    Point3 second;
    Point3 third;
};

class CurvenetFaceRegionBuilder
{
public:
    static void build(
        CurvenetCutResult& cutResult
    );

    static void buildFullSurfacePartitions(
        CurvenetCutResult& cutResult,
        int expectedRegionCount = -1
    );

    static void buildAuthoredSurfacePartitions(
        CurvenetCutResult& cutResult,
        int expectedFaceCount
    );

    static void buildTransferredLogicalPartitions(
        CurvenetCutResult& cutResult,
        const std::vector<TransferredRegionTriangle>& sourceTriangles,
        int expectedFaceCount
    );
};
