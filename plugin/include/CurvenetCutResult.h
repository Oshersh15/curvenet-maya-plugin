#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "CutPathMeshSplitter.h"
#include "SharedCurvenetNode.h"
#include "CurvenetFace.h"
#include "CurvenetEdge.h"
#include "ProfileCurveSampler.h"

struct EmbeddedSegmentVertex
{
    double segmentT = 0.0;

    int meshVertexId = -1;
};

struct SurfaceTrackingFailure
{
    int curveId = -1;
    int crossingCount = 0;
    int intervalCount = 0;
    int invalidIntervalCount = 0;
};

struct CurvenetCutResult
{
    bool success = false;

    HalfEdgeMesh mesh;
    std::unordered_set<int> embeddedVertexIds;
    std::unordered_set<int> embeddedHalfEdgeIds;
    std::unordered_set<int> embeddedFaceIds;

    std::vector<CutPathSplitResult>
        profileResults;

    std::vector<CutPath> attemptedCutPaths;

    int failedCurveId = -1;

    CutPathSplitFailure failedSplitReason =
        CutPathSplitFailure::None;

    int failedIntervalIndex = -1;
    int failedFirstVertexId = -1;
    int failedSecondVertexId = -1;

    int surfaceTrackedCurveCount = 0;
    std::vector<SurfaceTrackingFailure> surfaceTrackingFailures;

    std::unordered_map<int, CutChain>
        cutChainsByCurveId;

    /* Neutral authored geometry used to order curves at logical nodes. */
    std::unordered_map<int, std::vector<PolylineSegment>>
        sampledSegmentsByCurveId;

    /*
        Stores all embedded vertices created from each
        sampled curve segment.

        curveId
            -> segmentId
                -> candidate embedded vertices
    */
    std::unordered_map<
        int,
        std::unordered_map<
            int,
            std::vector<EmbeddedSegmentVertex>
        >
    > embeddedVerticesByCurveAndSegment;

    std::vector<SharedCurvenetNode> sharedCurvenetNodes;

    std::vector<CurvenetFace> curvenetFaces;

    int fullSurfaceRegionCountBeforeCleanup = -1;
    std::vector<int> mergedFullSurfaceRegionPolygonCounts;
    std::vector<double> mergedFullSurfaceRegionAreas;

    std::vector<CurvenetEdge> curvenetEdges;
};
