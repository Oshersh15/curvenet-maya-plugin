#pragma once

#include "CutChain.h"
#include "HalfEdge.h"

#include <unordered_map>
#include <vector>

class CurvenetHarmonicSolver
{
public:
    void initialize(
        const HalfEdgeMesh& cutMesh,
        const std::unordered_map<int, CutChain>& cutChainsByCurveId,
        int originalVertexCount,
        const std::vector<std::vector<Point3>>& neutralSampledCurves
    );

    std::vector<Point3> solve(
        const std::vector<std::vector<Point3>>& currentSampledCurves,
        int iterationCount = 60
    );

    bool isInitialized() const;

private:
    struct CurveConstraint
    {
        int curveId = -1;
        int segmentId = -1;
        double segmentT = 0.0;
    };

    int outputVertexCount = 0;
    std::vector<std::vector<int>> adjacentVertexIds;
    std::vector<std::vector<CurveConstraint>> constraintsByVertex;
    std::vector<std::vector<Point3>> neutralCurves;
    std::vector<Point3> displacements;
};
