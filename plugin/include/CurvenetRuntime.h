#pragma once

/* Defines lightweight Maya runtime objects for Curvenet controls and edges. */

#include <vector>

#include <maya/MObject.h>
#include <maya/MPoint.h>

struct CurvenetRuntimeNode
{
    int id = -1;

    MPoint position;

    MObject controlObject;
};

struct CurvenetRuntimeEdge
{
    int curveId = -1;

    int startNodeId = -1;

    int endNodeId = -1;
};

struct CurvenetRuntime
{
    std::vector<CurvenetRuntimeNode> nodes;

    std::vector<CurvenetRuntimeEdge> edges;
};
