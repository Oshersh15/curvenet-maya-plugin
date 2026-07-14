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
#include "VertexCurveBinding.h"

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

    currentSampledCurves.clear();

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

        int sampleCount = 0;

        if (!neutralSamplesCaptured)
        {
            sampleCount =
                ProfileCurveSampler::computeAdaptiveSampleCount(
                    controlPolygonLength,
                    meanMeshEdgeLength,
                    densityMultiplier
                );
        }
        else if (curveIndex < neutralSampledCurves.size())
        {
            sampleCount =
                static_cast<int>(
                    neutralSampledCurves[curveIndex].size()
                );
        }

        std::vector<Point3> sampledPoints =
            ProfileCurveSampler::sampleByArcLength(
                densePoints,
                sampleCount
            );

        currentSampledCurves.push_back(sampledPoints);

        if (!neutralSamplesCaptured)
        {
            neutralSampledCurves.push_back(sampledPoints);
        }

        debugSampledCurves.push_back(sampledPoints);

        std::vector<PolylineSegment> sampledSegments =
            ProfileCurveSampler::buildPolylineSegments(sampledPoints);

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

        cutPath.influencedVertexIds =
            mayaHalfEdgeMesh.collectUniqueVerticesFromFaces(
                cutPath.influencedFaceIds
            );

        if (!vertexBindingsCaptured)
        {
            for (int vertexId : cutPath.influencedVertexIds)
            {
                if (vertexId < 0 ||
                    vertexId >= static_cast<int>(
                        mayaHalfEdgeMesh.vertices.size()
                    ))
                {
                    continue;
                }

                const Point3& vertexPosition =
                    mayaHalfEdgeMesh.vertices[vertexId].position;

                ClosestCurveSegmentResult closestSegment =
                    GeometryUtils::findClosestPolylineSegment(
                        vertexPosition,
                        sampledSegments
                    );

                if (!closestSegment.found)
                {
                    continue;
                }

                VertexCurveBinding binding;

                binding.vertexId = vertexId;
                binding.curveId =
                    static_cast<int>(curveIndex);
                binding.segmentId =
                    closestSegment.segmentId;
                binding.segmentT =
                    closestSegment.segmentT;

                binding.neutralOffset =
                    GeometryUtils::subtract(
                        vertexPosition,
                        closestSegment.closestPoint
                    );

                vertexBindings.push_back(binding);
            }
        }

        cutPaths.push_back(cutPath);

        debugCrossings.insert(
            debugCrossings.end(),
            crossings.begin(),
            crossings.end()
        );
    }

    if (!vertexBindingsCaptured)
    {
        vertexBindingsCaptured = true;

        MGlobal::displayInfo(
            MString("Vertex bindings captured: ")
            + static_cast<int>(vertexBindings.size())
        );
    }

    if (!neutralSamplesCaptured)
    {
        neutralSamplesCaptured = true;

        MGlobal::displayInfo(
            "Neutral sampled curves captured."
        );
    }

    geoIterator.reset();

    while (!geoIterator.isDone())
    {
        const int vertexId =
            static_cast<int>(geoIterator.index());

        const VertexCurveBinding* matchingBinding =
            nullptr;

        for (const VertexCurveBinding& binding : vertexBindings)
        {
            if (binding.vertexId == vertexId)
            {
                matchingBinding = &binding;
                break;
            }
        }

        if (matchingBinding == nullptr)
        {
            geoIterator.next();
            continue;
        }

        const int curveId =
            matchingBinding->curveId;

        const int segmentId =
            matchingBinding->segmentId;

        if (curveId < 0 ||
            curveId >= static_cast<int>(
                neutralSampledCurves.size()
            ) ||
            curveId >= static_cast<int>(
                currentSampledCurves.size()
            ))
        {
            geoIterator.next();
            continue;
        }

        const std::vector<Point3>& neutralPoints =
            neutralSampledCurves[curveId];

        const std::vector<Point3>& currentPoints =
            currentSampledCurves[curveId];

        if (segmentId < 0 ||
            segmentId + 1 >= static_cast<int>(
                neutralPoints.size()
            ) ||
            segmentId + 1 >= static_cast<int>(
                currentPoints.size()
            ))
        {
            geoIterator.next();
            continue;
        }

        const Point3 displacement =
            GeometryUtils::interpolateSegmentDisplacement(
                neutralPoints[segmentId],
                neutralPoints[segmentId + 1],
                currentPoints[segmentId],
                currentPoints[segmentId + 1],
                matchingBinding->segmentT
            );

        MPoint vertexPosition =
            geoIterator.position();

        vertexPosition.x += displacement.x;
        vertexPosition.y += displacement.y;
        vertexPosition.z += displacement.z;

        geoIterator.setPosition(vertexPosition);
        geoIterator.next();
    }

    curvenetData.detectConnections(0.001);

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
