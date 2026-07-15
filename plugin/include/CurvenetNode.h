#pragma once

#include "CurvenetData.h"
#include "CutCrossing.h"
#include "HalfEdge.h"

#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MStatus.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>
#include "VertexCurveBinding.h"

#include <vector>

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

    MStatus deform(
        MDataBlock& dataBlock,
        MItGeometry& geoIterator,
        const MMatrix& localToWorldMatrix,
        unsigned int geometryIndex
    ) override;

    /* Returns the sampled points for every connected profile curve. */
    const std::vector<std::vector<Point3>>&
    getDebugSampledCurves() const;

    /* Returns all distinct curve-mesh crossings detected by the node. */
    const std::vector<CutCrossing>&
    getDebugCrossings() const;

    /* Returns the endpoint connections detected between profile curves. */
    const std::vector<CurveConnection>&
    getDebugConnections() const;

    static MTypeId id;
    static MString nodeName;
    static MObject inputCurves;
    static MObject inputMesh;

private:
    CurvenetData curvenetData;

    std::vector<std::vector<Point3>> debugSampledCurves;
    std::vector<CutCrossing> debugCrossings;

    std::vector<std::vector<Point3>> neutralSampledCurves;
    bool neutralSamplesCaptured = false;

    std::vector<std::vector<Point3>> currentSampledCurves;

    std::vector<VertexCurveBinding> vertexBindings;
    bool vertexBindingsCaptured = false;
};
