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
#include <maya/MFnMesh.h>
#include "HalfEdge.h"
#include "MayaMeshConverter.h"

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

        inputMesh = typedAttr.create(
            "inputMesh",
            "im",
            MFnData::kMesh,
            &status
        );

        if (!status)
        {
            status.perror("Failed to create inputMesh");
            return status;
        }

        typedAttr.setStorable(false);
        typedAttr.setReadable(false);
        typedAttr.setWritable(true);
        typedAttr.setConnectable(true);

        status = addAttribute(inputMesh);

        if (!status)
        {
            status.perror("Failed to add inputMesh");
            return status;
        }

        status = attributeAffects(inputMesh, outputGeom);

        if (!status)
        {
            status.perror("Failed to set attributeAffects for inputMesh");
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

        MDataHandle meshHandle =
            dataBlock.inputValue(inputMesh, &status);

        if (status)
        {
            MObject meshObject = meshHandle.asMesh();

            if (!meshObject.isNull())
            {
                MFnMesh meshFn(meshObject);

                int vertexCount =
                    meshFn.numVertices();

                int faceCount =
                    meshFn.numPolygons();

                MGlobal::displayInfo(
                    MString("Mesh vertices: ")
                    + vertexCount);

                MGlobal::displayInfo(
                    MString("Mesh faces: ")
                    + faceCount);

                HalfEdgeMesh mayaHalfEdgeMesh =
                    MayaMeshConverter::buildFromMayaMesh(meshFn);

                std::vector<int> faceHalfEdges =
                    mayaHalfEdgeMesh.getFaceHalfEdges(0);

                MGlobal::displayInfo("Face 0 half-edges:");

                for (int halfEdgeIndex : faceHalfEdges)
                {
                    const HalfEdge& halfEdge =
                        mayaHalfEdgeMesh.halfEdges[halfEdgeIndex];

                    MGlobal::displayInfo(
                        MString("HE")
                        + halfEdgeIndex
                        + ": "
                        + halfEdge.startVertex
                        + " -> "
                        + halfEdge.endVertex
                    );
                }

                if (mayaHalfEdgeMesh.halfEdges.size() > 7)
                {
                    MGlobal::displayInfo(
                        MString("HE1 twin: ")
                        + mayaHalfEdgeMesh.halfEdges[1].twin
                    );

                    MGlobal::displayInfo(
                        MString("HE7 twin: ")
                        + mayaHalfEdgeMesh.halfEdges[7].twin
                    );
                }

                MGlobal::displayInfo(
                    MString("HalfEdgeMesh vertices: ")
                    + static_cast<int>(mayaHalfEdgeMesh.vertices.size()));

                MGlobal::displayInfo(
                    MString("HalfEdgeMesh faces: ")
                    + static_cast<int>(mayaHalfEdgeMesh.faces.size()));

                MGlobal::displayInfo(
                    MString("HalfEdgeMesh halfEdges: ")
                    + static_cast<int>(mayaHalfEdgeMesh.halfEdges.size()));
            }
        }

        curvenetData.detectConnections(0.001);

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

            MGlobal::displayInfo(
                MString("    Start: (")
                + curve.startPoint.x + ", "
                + curve.startPoint.y + ", "
                + curve.startPoint.z + ")"
            );

            MGlobal::displayInfo(
                MString("    End: (")
                + curve.endPoint.x + ", "
                + curve.endPoint.y + ", "
                + curve.endPoint.z + ")"
            );
        }

        const auto& connections = curvenetData.getConnections();

        for (const auto& connection : connections)
        {
            auto endpointToString = [](CurveEndpoint endpoint)
            {
                return endpoint == CurveEndpoint::Start ? "start" : "end";
            };

            MGlobal::displayInfo(
                MString("Connection found: Curve ")
                + connection.firstCurveId
                + " "
                + endpointToString(connection.firstEndpoint)
                + " -> Curve "
                + connection.secondCurveId
                + " "
                + endpointToString(connection.secondEndpoint)
            );
        }

        for (const auto& curve : curves)
        {
            std::vector<int> connected =
                curvenetData.getConnectedCurves(curve.id);

            MString message =
                MString("Curve ")
                + curve.id
                + " connected to: ";

            for (int id : connected)
            {
                message += id;
                message += " ";
            }

            MGlobal::displayInfo(message);
        }

        // print Curvenet connections here
        // print connected curves here

        HalfEdgeMesh mesh;
        mesh.createTestQuad();

        std::vector<int> traversal = mesh.traverseFace(0);

        MString traversalMessage("Face traversal: ");

        for (int edge : traversal)
        {
            traversalMessage += edge;
            traversalMessage += " ";
        }

        MGlobal::displayInfo(traversalMessage);

        return MS::kSuccess;
    }

    static MTypeId id;
    static MString nodeName;
    static MObject inputCurves;
    static MObject inputMesh;

private:
    CurvenetData curvenetData;
};

MTypeId CurveDeformerNode::id(0x001226C1);
MString CurveDeformerNode::nodeName("curvenetNode");
MObject CurveDeformerNode::inputCurves;
MObject CurveDeformerNode::inputMesh;

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
