#pragma once

#include "CurvenetData.h"
#include "CurvenetHarmonicSolver.h"
#include "CutCrossing.h"
#include "HalfEdge.h"

#include <maya/MDataBlock.h>
#include <maya/MDagPath.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MStatus.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>
#include "VertexCurveBinding.h"

#include <vector>
#include <array>
#include <atomic>
#include <unordered_map>

/*
    Maya deformer node that stores the Curvenet representation, samples
    connected profile curves, and detects curve-mesh crossings.

    The node also caches read-only debug data so an explicit visualiser
    command can create viewport geometry without modifying the scene from
    inside deform().
*/
class CurveDeformerNode : public MPxDeformerNode
{
public:
    CurveDeformerNode() = default;
    ~CurveDeformerNode() override = default;

    static void* creator();
    static MStatus initialize();

    void postConstructor() override;

    SchedulingType schedulingType() const override;

    MStatus deform(
        MDataBlock& dataBlock,
        MItGeometry& geoIterator,
        const MMatrix& localToWorldMatrix,
        unsigned int geometryIndex
    ) override;

    MStatus prepareEmbedding(
        const MDagPath& meshPath,
        const std::vector<std::vector<Point3>>& profilePoints,
        const std::vector<int>& startNodeIds,
        const std::vector<int>& endNodeIds,
        bool fullSurface
    );

    /* Installs a fully computed cache without running embedding work on the
       Maya-owned dependency node. */
    void installPreparedEmbedding(CurveDeformerNode&& preparedNode);

    void reportPreparedEmbedding() const;

    /* Returns the sampled points for every connected profile curve. */
    const std::vector<std::vector<Point3>>&
    getDebugSampledCurves() const;

    /* Returns all distinct curve-mesh crossings detected by the node. */
    const std::vector<CutCrossing>&
    getDebugCrossings() const;

    /* Returns the endpoint connections detected between profile curves. */
    const std::vector<CurveConnection>&
    getDebugConnections() const;

    /* Returns the profile curves stored in the Curvenet representation. */
    const std::vector<ProfileCurveData>&
    getDebugProfileCurves() const;

    static MTypeId id;
    static MString nodeName;
    static MObject inputCurves;
    static MObject inputCurveCoordinates;
    static MObject inputCurveStartNodeIds;
    static MObject inputCurveEndNodeIds;
    static MObject inputDriverCurve;
    static MObject inputDriverNodeIds;
    static MObject inputMesh;
    static MObject fullSurfaceCurvenet;
    static MObject showGeneratedCurvenet;

private:
    MStatus evaluatePreparedState(
        MDataBlock* dataBlock,
        MItGeometry* geoIterator,
        const MMatrix& localToWorldMatrix,
        unsigned int geometryIndex,
        const MDagPath* preparationMeshPath = nullptr,
        const std::vector<std::vector<Point3>>* preparationProfilePoints = nullptr,
        const std::vector<int>* preparationStartNodeIds = nullptr,
        const std::vector<int>* preparationEndNodeIds = nullptr,
        const bool* preparationFullSurface = nullptr
    );

    std::atomic_bool deformInProgress{false};

    CurvenetData curvenetData;

    std::vector<std::vector<Point3>> debugSampledCurves;
    std::vector<CutCrossing> debugCrossings;

    std::vector<std::vector<Point3>> neutralSampledCurves;
    bool neutralSamplesCaptured = false;

    std::vector<std::vector<Point3>> currentSampledCurves;

    std::unordered_map<int, Point3> neutralDriverPositions;
    std::unordered_map<int, std::array<Point3, 4>> neutralDriverFrames;
    bool neutralDriverCaptured = false;

    std::vector<VertexCurveBinding> vertexBindings;
    CurvenetHarmonicSolver harmonicSolver;
    bool vertexBindingsCaptured = false;
    bool topologyCaptured = false;

    std::vector<MString> preparedInfoMessages;
    std::vector<MString> preparedWarningMessages;
    std::vector<MString> preparedErrorMessages;
};
