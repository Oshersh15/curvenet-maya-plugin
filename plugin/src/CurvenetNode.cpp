#include "CurvenetData.h"

#include <maya/MFnPlugin.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MItGeometry.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MPoint.h>
#include <maya/MTypeId.h>
#include <maya/MStatus.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MGlobal.h>
#include <maya/MString.h>
#include <maya/MObject.h>
#include <maya/MMatrix.h>

#include <vector>

class CurveDeformerNode : public MPxDeformerNode
{
public:
    CurveDeformerNode() = default;
    ~CurveDeformerNode() override = default;

    static void* creator()
    {
        return new CurveDeformerNode();
    }

    static MStatus initialize()
    {
        MStatus status;

        MFnTypedAttribute typedAttr;

        inputCurves = typedAttr.create(
            "inputCurves",
            "ics",
            MFnData::kNurbsCurve,
            &status
        );

        if (!status)
        {
            status.perror("Failed to create inputCurves attribute");
            return status;
        }

        typedAttr.setStorable(false);
        typedAttr.setReadable(false);
        typedAttr.setWritable(true);
        typedAttr.setConnectable(true);
        typedAttr.setArray(true);
        typedAttr.setUsesArrayDataBuilder(true);

        status = addAttribute(inputCurves);

        if (!status)
        {
            status.perror("Failed to add inputCurves attribute");
            return status;
        }

        status = attributeAffects(inputCurves, outputGeom);

        if (!status)
        {
            status.perror("Failed to set attributeAffects for inputCurves");
            return status;
        }

        return MS::kSuccess;
    }

    MStatus deform(
        MDataBlock& dataBlock,
        MItGeometry& geoIterator,
        const MMatrix& localToWorldMatrix,
        unsigned int geometryIndex
    ) override
    {
        MStatus status;

        curvenetData.clear();

        MArrayDataHandle curveArrayHandle =
            dataBlock.inputArrayValue(inputCurves, &status);

        if (!status)
        {
            return MS::kSuccess;
        }

        unsigned int numConnectedCurves = curveArrayHandle.elementCount();

        for (unsigned int curveIndex = 0; curveIndex < numConnectedCurves; ++curveIndex)
        {
            status = curveArrayHandle.jumpToArrayElement(curveIndex);

            if (!status)
            {
                continue;
            }

            MDataHandle curveHandle = curveArrayHandle.inputValue(&status);

            if (!status)
            {
                continue;
            }

            MObject curveObject = curveHandle.asNurbsCurve();

            if (curveObject.isNull())
            {
                continue;
            }

            MFnNurbsCurve curveFn(curveObject, &status);

            if (!status)
            {
                continue;
            }

            std::vector<MPoint> cvPositions;

            unsigned int numCVs = curveFn.numCVs();

            for (unsigned int cvIndex = 0; cvIndex < numCVs; ++cvIndex)
            {
                MPoint cvPosition;
                curveFn.getCV(cvIndex, cvPosition);
                cvPositions.push_back(cvPosition);
            }

            curvenetData.addCurve(curveObject, cvPositions);
        }

        MGlobal::displayInfo(
            MString("Curvenet contains ")
            + curvenetData.getCurveCount()
            + " profile curves."
        );

        const auto& curves = curvenetData.getCurves();

        for (const auto& curve : curves)
        {
            MGlobal::displayInfo(
                MString("Curve ")
                + curve.id
                + " has "
                + static_cast<int>(curve.restCVPositions.size())
                + " CVs."
            );
        }

        return MS::kSuccess;
    }

    static MTypeId id;
    static MString nodeName;
    static MObject inputCurves;

private:
    CurvenetData curvenetData;
};

MTypeId CurveDeformerNode::id(0x001226C1);
MString CurveDeformerNode::nodeName("curvenetNode");
MObject CurveDeformerNode::inputCurves;

MStatus initializePlugin(MObject pluginObject)
{
    MStatus status;
    MFnPlugin plugin(pluginObject, "Osher", "1.0", "Any");

    status = plugin.registerNode(
        CurveDeformerNode::nodeName,
        CurveDeformerNode::id,
        CurveDeformerNode::creator,
        CurveDeformerNode::initialize,
        MPxNode::kDeformerNode
    );

    if (!status)
    {
        status.perror("Failed to register curvenetNode");
    }

    return status;
}

MStatus uninitializePlugin(MObject pluginObject)
{
    MStatus status;
    MFnPlugin plugin(pluginObject);

    status = plugin.deregisterNode(CurveDeformerNode::id);

    if (!status)
    {
        status.perror("Failed to deregister curvenetNode");
    }

    return status;
}
