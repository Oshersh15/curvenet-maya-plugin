#include "CurvenetNode.h"
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
#include "ProfileCurveSampler.h"
#include <maya/MPxCommand.h>
#include <maya/MSelectionList.h>
#include <maya/MItSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include "ProfileCurveSampler.h"
#include "GeometryUtils.h"
#include "CurveMeshIntersector.h"
#include "CurvenetDebugCommand.h"
#include "CutPath.h"

#include <vector>

namespace
{
    bool getSelectedNurbsCurvePath(MDagPath& curvePath)
    {
        MStatus status;

        MSelectionList selection;
        status = MGlobal::getActiveSelectionList(selection);

        if (!status || selection.length() == 0)
        {
            MGlobal::displayError("Please select a NURBS curve.");
            return false;
        }

        MItSelectionList iterator(selection, MFn::kDagNode, &status);

        for (; !iterator.isDone(); iterator.next())
        {
            MDagPath selectedPath;
            status = iterator.getDagPath(selectedPath);

            if (!status)
            {
                continue;
            }

            if (selectedPath.hasFn(MFn::kNurbsCurve))
            {
                curvePath = selectedPath;
                return true;
            }

            if (selectedPath.hasFn(MFn::kTransform))
            {
                MFnDagNode dagNode(selectedPath, &status);

                if (!status)
                {
                    continue;
                }

                for (unsigned int childIndex = 0; childIndex < dagNode.childCount(); ++childIndex)
                {
                    MObject child = dagNode.child(childIndex, &status);

                    if (!status)
                    {
                        continue;
                    }

                    if (child.hasFn(MFn::kNurbsCurve))
                    {
                        curvePath = selectedPath;
                        curvePath.push(child);
                        return true;
                    }
                }
            }
        }

        MGlobal::displayError("Selection does not contain a NURBS curve.");
        return false;
    }


    void deleteExistingSampleLocators()
    {
        MGlobal::executeCommand(
            "string $sampleLocators[] = `ls \"profileCurveSample_*\"`; "
            "if (size($sampleLocators) > 0) delete $sampleLocators;",
            false,
            false
        );
    }

    void createLocatorAtPoint(const Point3& point, int index)
    {
        MString command;

        command += "spaceLocator -name \"profileCurveSample_";
        command += index;
        command += "\" -position ";
        command += point.x;
        command += " ";
        command += point.y;
        command += " ";
        command += point.z;
        command += ";";

        MGlobal::executeCommand(command, false, false);
    }
}

std::vector<Point3> buildDenseCurvePoints(
    MFnNurbsCurve& curveFn,
    int sampleCount
)
{
    std::vector<Point3> densePoints;

    if (sampleCount < 2)
    {
        return densePoints;
    }

    double minParam = 0.0;
    double maxParam = 0.0;

    MStatus status = curveFn.getKnotDomain(minParam, maxParam);

    if (!status)
    {
        return densePoints;
    }

    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
    {
        double ratio =
            static_cast<double>(sampleIndex) /
            static_cast<double>(sampleCount - 1);

        double parameter =
            minParam + (maxParam - minParam) * ratio;

        MPoint mayaPoint;

        status = curveFn.getPointAtParam(
            parameter,
            mayaPoint,
            MSpace::kWorld
        );

        if (!status)
        {
            continue;
        }

        densePoints.push_back(Point3{
            mayaPoint.x,
            mayaPoint.y,
            mayaPoint.z
        });
    }

    return densePoints;
}

void* CurveDeformerNode::creator()
{
    return new CurveDeformerNode();
}

MStatus CurveDeformerNode::initialize()
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

MStatus CurveDeformerNode::deform(
MDataBlock& dataBlock,
MItGeometry& geoIterator,
const MMatrix& localToWorldMatrix,
unsigned int geometryIndex
)
{
    MStatus status;

    curvenetData.clear();
    debugSampledCurves.clear();
    debugCrossings.clear();

    double meanMeshEdgeLength = 0.0;
    HalfEdgeMesh mayaHalfEdgeMesh;

    MDataHandle meshHandle =
        dataBlock.inputValue(inputMesh, &status);

    if (status)
    {
        MObject meshObject = meshHandle.asMesh();

        if (!meshObject.isNull())
        {
            MFnMesh meshFn(meshObject);

            mayaHalfEdgeMesh =
                MayaMeshConverter::buildFromMayaMesh(meshFn);

            meanMeshEdgeLength =
                mayaHalfEdgeMesh.computeMeanEdgeLength();

            MGlobal::displayInfo(
                MString("Mean mesh edge length: ")
                + meanMeshEdgeLength
            );
        }
    }

    MArrayDataHandle curveArrayHandle =
        dataBlock.inputArrayValue(inputCurves, &status);

    if (!status)
    {
        return MS::kSuccess;
    }

    unsigned int numConnectedCurves = curveArrayHandle.elementCount();

    std::vector<CutPath> cutPaths;

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
        std::vector<Point3> controlPoints;

        unsigned int numCVs = curveFn.numCVs();

        for (unsigned int cvIndex = 0; cvIndex < numCVs; ++cvIndex)
        {
            MPoint cvPosition;
            curveFn.getCV(cvIndex, cvPosition);
            cvPositions.push_back(cvPosition);

            controlPoints.push_back(Point3{
                cvPosition.x,
                cvPosition.y,
                cvPosition.z
            });
        }

        curvenetData.addCurve(curveObject, cvPositions);

        std::vector<Point3> densePoints =
            buildDenseCurvePoints(curveFn, 200);

        const double controlPolygonLength =
            ProfileCurveSampler::computeControlPolygonLength(controlPoints);

        const int densityMultiplier = 5;

        const int adaptiveSampleCount =
            ProfileCurveSampler::computeAdaptiveSampleCount(
                controlPolygonLength,
                meanMeshEdgeLength,
                densityMultiplier
            );

        MGlobal::displayInfo(
            MString("Control polygon length: ")
            + controlPolygonLength
        );

        MGlobal::displayInfo(
            MString("Adaptive sample count: ")
            + adaptiveSampleCount
        );

        std::vector<Point3> sampledPoints =
            ProfileCurveSampler::sampleByArcLength(
                densePoints,
                adaptiveSampleCount
            );

        debugSampledCurves.push_back(sampledPoints);

        std::vector<PolylineSegment> sampledSegments =
            ProfileCurveSampler::buildPolylineSegments(sampledPoints);

        for (int segmentIndex = 0;
             segmentIndex < static_cast<int>(sampledSegments.size());
             ++segmentIndex)
        {
            const PolylineSegment& segment =
                sampledSegments[segmentIndex];

            MGlobal::displayInfo(
                MString("Segment ")
                + segmentIndex
                + ": ("
                + segment.start.x + ", "
                + segment.start.y + ", "
                + segment.start.z + ") -> ("
                + segment.end.x + ", "
                + segment.end.y + ", "
                + segment.end.z + ")"
            );
        }

        const double crossingTolerance = 0.0501;

        const double duplicateTolerance = 0.0001;

        std::vector<CutCrossing> crossings =
            CurveMeshIntersector::findAllCrossings(
                static_cast<int>(curveIndex),
                sampledSegments,
                mayaHalfEdgeMesh,
                crossingTolerance,
                duplicateTolerance
            );

        CutPath cutPath;

        cutPath.curveId =
            static_cast<int>(curveIndex);

        cutPath.crossings = crossings;

        std::vector<int> faceIntervals =
            CurveMeshIntersector::deriveFaceIntervals(
                cutPath,
                mayaHalfEdgeMesh
            );

        std::vector<int> influencedFaceIds =
            CurveMeshIntersector::collectUniqueFaces(
                faceIntervals
            );

        cutPath.influencedFaceIds =
            influencedFaceIds;

        MGlobal::displayInfo(
            MString("Influenced faces: ")
            + static_cast<int>(
                cutPath.influencedFaceIds.size()
            )
        );

        cutPath.influencedVertexIds =
            mayaHalfEdgeMesh.collectUniqueVerticesFromFaces(
                cutPath.influencedFaceIds
            );

        MGlobal::displayInfo(
            MString("Influenced vertices: ")
            + static_cast<int>(
                cutPath.influencedVertexIds.size()
            )
        );

        for (int faceId : cutPath.influencedFaceIds)
        {
            MGlobal::displayInfo(
                MString("  Face ")
                + faceId
            );
        }

        cutPaths.push_back(cutPath);

        MGlobal::displayInfo(
            MString("CutPath curve ID: ")
            + cutPath.curveId
        );

        MGlobal::displayInfo(
            MString("CutPath crossings: ")
            + static_cast<int>(cutPath.crossings.size())
        );

        debugCrossings.insert(
            debugCrossings.end(),
            crossings.begin(),
            crossings.end()
        );

        MGlobal::displayInfo(
            MString("Crossings found: ")
            + static_cast<int>(crossings.size())
        );

        for (const CutCrossing& crossing : crossings)
        {
            MGlobal::displayInfo(
                MString("Crossing:")
                + " curve "
                + crossing.curveId
                + ", curve segment "
                + crossing.curveSegmentId
                + ", face "
                + crossing.faceId
                + ", half-edge "
                + crossing.halfEdgeId
                + ", position ("
                + crossing.position.x + ", "
                + crossing.position.y + ", "
                + crossing.position.z + ")"
            );
        }

        MGlobal::displayInfo(
            MString("CutPaths created: ")
            + static_cast<int>(cutPaths.size())
        );

        MGlobal::displayInfo(
            MString("Dense curve points: ")
            + static_cast<int>(densePoints.size())
        );

        MGlobal::displayInfo(
            MString("Arc-length sampled points: ")
            + static_cast<int>(sampledPoints.size())
        );
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

const std::vector<std::vector<Point3>>&
CurveDeformerNode::getDebugSampledCurves() const
{
    return debugSampledCurves;
}

const std::vector<CutCrossing>&
CurveDeformerNode::getDebugCrossings() const
{
    return debugCrossings;
}


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

    status = plugin.registerCommand(
        CurvenetDebugCommand::commandName,
        CurvenetDebugCommand::creator
    );

    if (!status)
    {
        status.perror(
            "Failed to register visualizeCurvenetDebug command"
        );

        return status;
    }

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

    status = plugin.deregisterCommand(
        CurvenetDebugCommand::commandName
    );

    if (!status)
    {
        status.perror(
            "Failed to deregister visualizeCurvenetDebug command"
        );

        return status;
    }

    if (!status)
    {
        status.perror("Failed to deregister sampleProfileCurve command");
    }

    status = plugin.deregisterNode(CurveDeformerNode::id);

    if (!status)
    {
        status.perror("Failed to deregister curvenetNode");
    }

    return status;
}
