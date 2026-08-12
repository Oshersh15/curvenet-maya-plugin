#include "CurvenetNode.h"
#include "CurvenetData.h"

#include <maya/MFnPlugin.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MItGeometry.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MTypeId.h>
#include <maya/MStatus.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MGlobal.h>
#include <maya/MString.h>
#include <maya/MObject.h>
#include <maya/MMatrix.h>
#include <maya/MFnMesh.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnGeometryFilter.h>
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
#include "CutPathMeshSplitter.h"
#include "CurvenetFaceBuilder.h"
#include "CurvenetFaceRegionBuilder.h"
#include "CurvenetMeshCutter.h"
#include "CurvenetEdgeBuilder.h"

#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace
{
    struct LocalNodeTransform
    {
        std::array<double, 9> rotation = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
        Point3 neutralAnchor;
        Point3 currentAnchor;
    };

    Point3 crossProduct(const Point3& first, const Point3& second)
    {
        return Point3{
            first.y * second.z - first.z * second.y,
            first.z * second.x - first.x * second.z,
            first.x * second.y - first.y * second.x
        };
    }

    Point3 normalized(const Point3& value)
    {
        const double length = std::sqrt(
            value.x * value.x + value.y * value.y + value.z * value.z
        );

        if (length <= 1.0e-12)
        {
            return Point3{};
        }

        return Point3{
            value.x / length,
            value.y / length,
            value.z / length
        };
    }

    Point3 applyLocalTransform(
        const LocalNodeTransform& transform,
        const Point3& point
    )
    {
        const Point3 relative{
            point.x - transform.neutralAnchor.x,
            point.y - transform.neutralAnchor.y,
            point.z - transform.neutralAnchor.z
        };

        return Point3{
            transform.currentAnchor.x +
                transform.rotation[0] * relative.x +
                transform.rotation[1] * relative.y +
                transform.rotation[2] * relative.z,
            transform.currentAnchor.y +
                transform.rotation[3] * relative.x +
                transform.rotation[4] * relative.y +
                transform.rotation[5] * relative.z,
            transform.currentAnchor.z +
                transform.rotation[6] * relative.x +
                transform.rotation[7] * relative.y +
                transform.rotation[8] * relative.z
        };
    }

    LocalNodeTransform buildLocalNodeTransform(
        int nodeId,
        const std::vector<int>& neighbours,
        const std::unordered_map<int, Point3>& neutralPositions,
        const std::unordered_map<int, Point3>& currentPositions
    )
    {
        LocalNodeTransform transform;
        const auto neutralCentre = neutralPositions.find(nodeId);
        const auto currentCentre = currentPositions.find(nodeId);

        if (neutralCentre == neutralPositions.end() ||
            currentCentre == currentPositions.end())
        {
            return transform;
        }

        transform.neutralAnchor = neutralCentre->second;
        transform.currentAnchor = currentCentre->second;
        int firstNeighbour = -1;
        double firstLengthSquared = 0.0;

        for (int neighbourId : neighbours)
        {
            const auto neutralNeighbour = neutralPositions.find(neighbourId);
            const auto currentNeighbour = currentPositions.find(neighbourId);

            if (neutralNeighbour == neutralPositions.end() ||
                currentNeighbour == currentPositions.end())
            {
                continue;
            }

            const Point3 direction = GeometryUtils::subtract(
                neutralNeighbour->second,
                neutralCentre->second
            );
            const double lengthSquared =
                direction.x * direction.x +
                direction.y * direction.y +
                direction.z * direction.z;

            if (lengthSquared > firstLengthSquared)
            {
                firstLengthSquared = lengthSquared;
                firstNeighbour = neighbourId;
            }
        }

        if (firstNeighbour < 0)
        {
            return transform;
        }

        const Point3 neutralFirst = GeometryUtils::subtract(
            neutralPositions.at(firstNeighbour),
            neutralCentre->second
        );
        int secondNeighbour = -1;
        double bestCrossLengthSquared = 0.0;

        for (int neighbourId : neighbours)
        {
            if (neighbourId == firstNeighbour ||
                neutralPositions.find(neighbourId) == neutralPositions.end() ||
                currentPositions.find(neighbourId) == currentPositions.end())
            {
                continue;
            }

            const Point3 candidate = GeometryUtils::subtract(
                neutralPositions.at(neighbourId),
                neutralCentre->second
            );
            const Point3 cross = crossProduct(neutralFirst, candidate);
            const double crossLengthSquared =
                cross.x * cross.x + cross.y * cross.y + cross.z * cross.z;

            if (crossLengthSquared > bestCrossLengthSquared)
            {
                bestCrossLengthSquared = crossLengthSquared;
                secondNeighbour = neighbourId;
            }
        }

        if (secondNeighbour < 0 || bestCrossLengthSquared <= 1.0e-16)
        {
            return transform;
        }

        const Point3 neutralSecond = GeometryUtils::subtract(
            neutralPositions.at(secondNeighbour),
            neutralCentre->second
        );
        const Point3 currentFirst = GeometryUtils::subtract(
            currentPositions.at(firstNeighbour),
            currentCentre->second
        );
        const Point3 currentSecond = GeometryUtils::subtract(
            currentPositions.at(secondNeighbour),
            currentCentre->second
        );
        const Point3 neutralX = normalized(neutralFirst);
        const Point3 neutralZ = normalized(
            crossProduct(neutralFirst, neutralSecond)
        );
        const Point3 neutralY = crossProduct(neutralZ, neutralX);
        const Point3 currentX = normalized(currentFirst);
        const Point3 currentZ = normalized(
            crossProduct(currentFirst, currentSecond)
        );
        const Point3 currentY = crossProduct(currentZ, currentX);
        const Point3 neutralBasis[3] = {neutralX, neutralY, neutralZ};
        const Point3 currentBasis[3] = {currentX, currentY, currentZ};

        for (int row = 0; row < 3; ++row)
        {
            const double currentValues[3] = {
                row == 0 ? currentBasis[0].x :
                    (row == 1 ? currentBasis[0].y : currentBasis[0].z),
                row == 0 ? currentBasis[1].x :
                    (row == 1 ? currentBasis[1].y : currentBasis[1].z),
                row == 0 ? currentBasis[2].x :
                    (row == 1 ? currentBasis[2].y : currentBasis[2].z)
            };

            for (int column = 0; column < 3; ++column)
            {
                const double neutralValues[3] = {
                    column == 0 ? neutralBasis[0].x :
                        (column == 1 ? neutralBasis[0].y : neutralBasis[0].z),
                    column == 0 ? neutralBasis[1].x :
                        (column == 1 ? neutralBasis[1].y : neutralBasis[1].z),
                    column == 0 ? neutralBasis[2].x :
                        (column == 1 ? neutralBasis[2].y : neutralBasis[2].z)
                };
                transform.rotation[row * 3 + column] =
                    currentValues[0] * neutralValues[0] +
                    currentValues[1] * neutralValues[1] +
                    currentValues[2] * neutralValues[2];
            }
        }

        return transform;
    }

    LocalNodeTransform buildEncodedNodeTransform(
        const std::array<Point3, 4>& neutralFrame,
        const std::vector<Point3>& currentFrame
    )
    {
        if (currentFrame.size() < 4)
        {
            return LocalNodeTransform{};
        }

        const std::unordered_map<int, Point3> neutralPositions = {
            {0, neutralFrame[0]}, {1, neutralFrame[1]},
            {2, neutralFrame[2]}, {3, neutralFrame[3]}
        };
        const std::unordered_map<int, Point3> currentPositions = {
            {0, currentFrame[0]}, {1, currentFrame[1]},
            {2, currentFrame[2]}, {3, currentFrame[3]}
        };
        return buildLocalNodeTransform(
            0,
            {1, 2, 3},
            neutralPositions,
            currentPositions
        );
    }

    int authoredCycleRank(
        const std::vector<ProfileCutInput>& profileInputs
    )
    {
        std::unordered_map<int, int> parent;

        for (const ProfileCutInput& input : profileInputs)
        {
            if (input.authoredStartNodeId < 0 ||
                input.authoredEndNodeId < 0)
            {
                return -1;
            }

            parent[input.authoredStartNodeId] =
                input.authoredStartNodeId;
            parent[input.authoredEndNodeId] =
                input.authoredEndNodeId;
        }

        const auto findRoot = [&parent](int nodeId)
        {
            int root = nodeId;

            while (parent[root] != root)
            {
                root = parent[root];
            }

            return root;
        };

        for (const ProfileCutInput& input : profileInputs)
        {
            const int startRoot = findRoot(input.authoredStartNodeId);
            const int endRoot = findRoot(input.authoredEndNodeId);

            if (startRoot != endRoot)
            {
                parent[endRoot] = startRoot;
            }
        }

        std::unordered_set<int> componentRoots;

        for (const auto& entry : parent)
        {
            componentRoots.insert(findRoot(entry.first));
        }

        return static_cast<int>(profileInputs.size()) -
            static_cast<int>(parent.size()) +
            static_cast<int>(componentRoots.size());
    }

    int meshBoundaryComponentCount(const HalfEdgeMesh& mesh)
    {
        std::unordered_map<int, std::vector<int>> boundaryNeighbours;

        for (const HalfEdge& halfEdge : mesh.halfEdges)
        {
            if (halfEdge.twin >= 0)
            {
                continue;
            }

            boundaryNeighbours[halfEdge.startVertex].push_back(
                halfEdge.endVertex
            );
            boundaryNeighbours[halfEdge.endVertex].push_back(
                halfEdge.startVertex
            );
        }

        std::unordered_set<int> visitedVertices;
        int componentCount = 0;

        for (const auto& entry : boundaryNeighbours)
        {
            if (!visitedVertices.insert(entry.first).second)
            {
                continue;
            }

            ++componentCount;
            std::vector<int> pendingVertices = {entry.first};

            while (!pendingVertices.empty())
            {
                const int vertexId = pendingVertices.back();
                pendingVertices.pop_back();

                for (int neighbourVertexId :
                     boundaryNeighbours[vertexId])
                {
                    if (visitedVertices.insert(neighbourVertexId).second)
                    {
                        pendingVertices.push_back(neighbourVertexId);
                    }
                }
            }
        }

        return componentCount;
    }

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
            MSpace::kObject
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

MPxNode::SchedulingType CurveDeformerNode::schedulingType() const
{
    /* Evaluation updates node-owned embedding and deformation caches. Keep
       separate Curvenet nodes from mutating those caches concurrently. */
    return MPxNode::kGloballySerial;
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

    MFnNumericAttribute numericAttr;

    inputCurveStartNodeIds = numericAttr.create(
        "inputCurveStartNodeIds",
        "icsn",
        MFnNumericData::kInt,
        -1,
        &status
    );
    numericAttr.setArray(true);
    numericAttr.setUsesArrayDataBuilder(true);
    numericAttr.setStorable(true);
    addAttribute(inputCurveStartNodeIds);
    attributeAffects(inputCurveStartNodeIds, outputGeom);

    inputCurveEndNodeIds = numericAttr.create(
        "inputCurveEndNodeIds",
        "icen",
        MFnNumericData::kInt,
        -1,
        &status
    );
    numericAttr.setArray(true);
    numericAttr.setUsesArrayDataBuilder(true);
    numericAttr.setStorable(true);
    addAttribute(inputCurveEndNodeIds);
    attributeAffects(inputCurveEndNodeIds, outputGeom);

    inputDriverCurve = typedAttr.create(
        "inputDriverCurve",
        "idc",
        MFnData::kNurbsCurve,
        &status
    );

    if (!status)
    {
        status.perror("Failed to create inputDriverCurve");
        return status;
    }

    typedAttr.setStorable(false);
    typedAttr.setReadable(false);
    typedAttr.setWritable(true);
    typedAttr.setConnectable(true);
    typedAttr.setArray(false);
    status = addAttribute(inputDriverCurve);

    if (!status)
    {
        status.perror("Failed to add inputDriverCurve");
        return status;
    }

    attributeAffects(inputDriverCurve, outputGeom);

    inputDriverNodeIds = numericAttr.create(
        "inputDriverNodeIds",
        "idn",
        MFnNumericData::kInt,
        -1,
        &status
    );
    numericAttr.setArray(true);
    numericAttr.setUsesArrayDataBuilder(true);
    numericAttr.setStorable(true);
    addAttribute(inputDriverNodeIds);
    attributeAffects(inputDriverNodeIds, outputGeom);

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

    fullSurfaceCurvenet = numericAttr.create(
        "fullSurfaceCurvenet",
        "fsc",
        MFnNumericData::kBoolean,
        true,
        &status
    );

    if (!status)
    {
        status.perror("Failed to create fullSurfaceCurvenet");
        return status;
    }

    numericAttr.setStorable(true);
    numericAttr.setKeyable(true);

    status = addAttribute(fullSurfaceCurvenet);

    if (!status)
    {
        status.perror("Failed to add fullSurfaceCurvenet");
        return status;
    }

    status = attributeAffects(fullSurfaceCurvenet, outputGeom);

    if (!status)
    {
        status.perror("Failed to set fullSurfaceCurvenet affects");
        return status;
    }

    showGeneratedCurvenet = numericAttr.create(
        "showGeneratedCurvenet",
        "sgc",
        MFnNumericData::kBoolean,
        false,
        &status
    );

    if (!status)
    {
        status.perror("Failed to create showGeneratedCurvenet");
        return status;
    }

    numericAttr.setStorable(true);
    numericAttr.setKeyable(true);

    status = addAttribute(showGeneratedCurvenet);

    if (!status)
    {
        status.perror("Failed to add showGeneratedCurvenet");
        return status;
    }

    status = attributeAffects(showGeneratedCurvenet, outputGeom);

    if (!status)
    {
        status.perror("Failed to set showGeneratedCurvenet affects");
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

    MMatrix geometryLocalToWorldMatrix =
        localToWorldMatrix;
    MString geometryTransformName;

    MFnGeometryFilter geometryFilter(
        thisMObject(),
        &status
    );

    if (status)
    {
        MDagPath geometryPath;

        status = geometryFilter.getPathAtIndex(
            geometryIndex,
            geometryPath
        );

        if (status)
        {
            geometryLocalToWorldMatrix =
                geometryPath.inclusiveMatrix();

            if (!geometryPath.hasFn(MFn::kTransform))
            {
                geometryPath.pop();
            }

            geometryTransformName =
                geometryPath.fullPathName();
        }
    }

    curvenetData.clear();
    debugSampledCurves.clear();
    debugCrossings.clear();

    double meanMeshEdgeLength = 0.0;
    HalfEdgeMesh mayaHalfEdgeMesh;

    /* The half-edge mesh is needed only while constructing the neutral
       embedding. Rebuilding it on every driver update made interactive
       posing scale with the full mesh even though the CutPaths were cached. */
    if (!topologyCaptured)
    {
        MArrayDataHandle geometryArray =
            dataBlock.inputArrayValue(input, &status);

        if (status && geometryArray.jumpToElement(geometryIndex))
        {
            MDataHandle geometryHandle =
                geometryArray.inputValue(&status);
            MDataHandle meshHandle =
                geometryHandle.child(inputGeom);
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
    }

    MArrayDataHandle curveArrayHandle =
        dataBlock.inputArrayValue(inputCurves, &status);

    if (!status)
    {
        return MS::kSuccess;
    }

    unsigned int numConnectedCurves = curveArrayHandle.elementCount();


    std::vector<CutPath> cutPaths;
    std::vector<ProfileCutInput> profileInputs;

    const auto authoredNodeId =
        [&dataBlock](const MObject& attribute, unsigned int index)
        {
            MStatus handleStatus;
            MArrayDataHandle handle =
                dataBlock.inputArrayValue(attribute, &handleStatus);

            if (!handleStatus || !handle.jumpToArrayElement(index))
            {
                return -1;
            }

            return handle.inputValue(&handleStatus).asInt();
        };

    std::unordered_map<int, Point3> currentDriverPositions;
    std::unordered_map<int, std::vector<Point3>> currentDriverFramePoints;
    MDataHandle driverHandle =
        dataBlock.inputValue(inputDriverCurve, &status);

    if (status)
    {
        const MObject driverObject =
            driverHandle.asNurbsCurve();

        if (!driverObject.isNull())
        {
            MFnNurbsCurve driverFn(driverObject, &status);

            if (status)
            {
                MArrayDataHandle driverIdHandle =
                    dataBlock.inputArrayValue(inputDriverNodeIds, &status);
                const MMatrix worldToLocalMatrix =
                    geometryLocalToWorldMatrix.inverse();
                MPointArray driverPositions;
                MStatus cvStatus =
                    driverFn.getCVs(
                        driverPositions,
                        MSpace::kObject
                    );

                for (unsigned int cvIndex = 0;
                     cvStatus && cvIndex < driverPositions.length();
                     ++cvIndex)
                {
                    if (!status ||
                        !driverIdHandle.jumpToArrayElement(cvIndex))
                    {
                        continue;
                    }

                    MStatus idStatus;
                    const int logicalNodeId =
                        driverIdHandle.inputValue(&idStatus).asInt();

                    if (!idStatus || logicalNodeId < 0)
                    {
                        continue;
                    }

                    MPoint position = driverPositions[cvIndex];
                    position *= worldToLocalMatrix;
                    currentDriverFramePoints[logicalNodeId].push_back(Point3{
                        position.x,
                        position.y,
                        position.z
                    });
                }
            }
        }
    }

    for (const auto& entry : currentDriverFramePoints)
    {
        if (entry.second.empty())
        {
            continue;
        }

        currentDriverPositions[entry.first] = entry.second.front();

        if (!neutralDriverCaptured && entry.second.size() >= 4)
        {
            neutralDriverFrames[entry.first] = {
                entry.second[0], entry.second[1],
                entry.second[2], entry.second[3]
            };
        }
    }

    if (!neutralDriverCaptured && !currentDriverPositions.empty())
    {
        neutralDriverPositions = currentDriverPositions;
        neutralDriverCaptured = true;
    }

    std::unordered_map<int, std::vector<int>> driverNeighbours;

    for (unsigned int curveIndex = 0;
         curveIndex < numConnectedCurves;
         ++curveIndex)
    {
        const int startNodeId =
            authoredNodeId(inputCurveStartNodeIds, curveIndex);
        const int endNodeId =
            authoredNodeId(inputCurveEndNodeIds, curveIndex);

        if (startNodeId < 0 || endNodeId < 0 || startNodeId == endNodeId)
        {
            continue;
        }

        driverNeighbours[startNodeId].push_back(endNodeId);
        driverNeighbours[endNodeId].push_back(startNodeId);
    }

    std::unordered_map<int, LocalNodeTransform> driverTransforms;

    for (auto& entry : driverNeighbours)
    {
        std::vector<int>& neighbours = entry.second;
        std::sort(neighbours.begin(), neighbours.end());
        neighbours.erase(
            std::unique(neighbours.begin(), neighbours.end()),
            neighbours.end()
        );
        driverTransforms[entry.first] = buildLocalNodeTransform(
            entry.first,
            neighbours,
            neutralDriverPositions,
            currentDriverPositions
        );
    }

    /* Orientation-aware drivers encode an origin and three skinned axis
       points per logical node. Prefer that exact Maya skin transformation
       over estimating rotation from neighbouring Curvenet nodes. */
    for (const auto& entry : currentDriverFramePoints)
    {
        const auto neutralFrame = neutralDriverFrames.find(entry.first);

        if (neutralFrame != neutralDriverFrames.end() &&
            entry.second.size() >= 4)
        {
            driverTransforms[entry.first] = buildEncodedNodeTransform(
                neutralFrame->second,
                entry.second
            );
        }
    }

    const auto applyDriverDisplacement = [
        &authoredNodeId,
        &currentDriverPositions,
        &driverTransforms,
        this
    ](
        unsigned int curveIndex,
        std::vector<Point3>& points
    )
    {
        const int startLogicalNodeId =
            authoredNodeId(inputCurveStartNodeIds, curveIndex);
        const int endLogicalNodeId =
            authoredNodeId(inputCurveEndNodeIds, curveIndex);
        const auto neutralStart =
            neutralDriverPositions.find(startLogicalNodeId);
        const auto neutralEnd =
            neutralDriverPositions.find(endLogicalNodeId);
        const auto currentStart =
            currentDriverPositions.find(startLogicalNodeId);
        const auto currentEnd =
            currentDriverPositions.find(endLogicalNodeId);
        const auto startTransform =
            driverTransforms.find(startLogicalNodeId);
        const auto endTransform =
            driverTransforms.find(endLogicalNodeId);

        if (neutralStart == neutralDriverPositions.end() ||
            neutralEnd == neutralDriverPositions.end() ||
            currentStart == currentDriverPositions.end() ||
            currentEnd == currentDriverPositions.end())
        {
            return;
        }

        LocalNodeTransform fallbackStart;
        fallbackStart.neutralAnchor = neutralStart->second;
        fallbackStart.currentAnchor = currentStart->second;
        LocalNodeTransform fallbackEnd;
        fallbackEnd.neutralAnchor = neutralEnd->second;
        fallbackEnd.currentAnchor = currentEnd->second;
        const LocalNodeTransform& resolvedStart =
            startTransform != driverTransforms.end()
                ? startTransform->second
                : fallbackStart;
        const LocalNodeTransform& resolvedEnd =
            endTransform != driverTransforms.end()
                ? endTransform->second
                : fallbackEnd;

        for (int pointIndex = 0;
             pointIndex < static_cast<int>(points.size());
             ++pointIndex)
        {
            const double parameter =
                points.size() > 1
                    ? static_cast<double>(pointIndex) /
                        static_cast<double>(points.size() - 1)
                    : 0.0;
            const Point3 transformedStart = applyLocalTransform(
                resolvedStart,
                points[pointIndex]
            );
            const Point3 transformedEnd = applyLocalTransform(
                resolvedEnd,
                points[pointIndex]
            );
            points[pointIndex] = Point3{
                transformedStart.x * (1.0 - parameter) +
                    transformedEnd.x * parameter,
                transformedStart.y * (1.0 - parameter) +
                    transformedEnd.y * parameter,
                transformedStart.z * (1.0 - parameter) +
                    transformedEnd.z * parameter
            };
        }
    };

    currentSampledCurves.clear();

    for (unsigned int curveIndex = 0; curveIndex < numConnectedCurves; ++curveIndex)
    {
        if (topologyCaptured &&
            !currentDriverPositions.empty() &&
            curveIndex < neutralSampledCurves.size())
        {
            std::vector<Point3> objectSampledPoints =
                neutralSampledCurves[curveIndex];
            applyDriverDisplacement(curveIndex, objectSampledPoints);
            currentSampledCurves.push_back(
                std::move(objectSampledPoints)
            );
            continue;
        }

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

        MObject curveObject =
            curveHandle.asNurbsCurve();

        if (curveObject.isNull())
        {
            continue;
        }

        MFnNurbsCurve curveFn(curveObject, &status);

        if (!status)
        {
            continue;
        }

        const MFnNurbsCurve::Form curveForm =
            curveFn.form();

        const bool curveClosed =
            curveForm == MFnNurbsCurve::kClosed ||
            curveForm == MFnNurbsCurve::kPeriodic;

        std::vector<MPoint> cvPositions;
        std::vector<Point3> controlPoints;

        MPointArray curveCVs;
        MStatus cvStatus =
            curveFn.getCVs(
                curveCVs,
                MSpace::kObject
            );

        if (!cvStatus)
        {
            continue;
        }

        for (unsigned int cvIndex = 0;
             cvIndex < curveCVs.length();
             ++cvIndex)
        {
            const MPoint cvPosition = curveCVs[cvIndex];
            cvPositions.push_back(cvPosition);

            controlPoints.push_back(Point3{
                cvPosition.x,
                cvPosition.y,
                cvPosition.z
            });
        }

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

        std::vector<Point3> objectSampledPoints;
        objectSampledPoints.reserve(sampledPoints.size());

        const MMatrix worldToLocalMatrix =
            geometryLocalToWorldMatrix.inverse();

        for (const Point3& sampledPoint : sampledPoints)
        {
            const MPoint objectPoint =
                MPoint(
                    sampledPoint.x,
                    sampledPoint.y,
                    sampledPoint.z
                ) * worldToLocalMatrix;

            objectSampledPoints.push_back(Point3{
                objectPoint.x,
                objectPoint.y,
                objectPoint.z
            });
        }

        applyDriverDisplacement(curveIndex, objectSampledPoints);

        curvenetData.addCurve(
            curveObject,
            cvPositions,
            sampledPoints,
            curveClosed
        );

        currentSampledCurves.push_back(objectSampledPoints);

        if (!neutralSamplesCaptured)
        {
            neutralSampledCurves.push_back(objectSampledPoints);
        }

        debugSampledCurves.push_back(sampledPoints);

        /*
            Cutting and region discovery describe the neutral embedding. Once
            captured, interactive evaluations only need the moving samples.
        */
        if (topologyCaptured)
        {
            continue;
        }

        std::vector<PolylineSegment> sampledSegments =
            ProfileCurveSampler::buildPolylineSegments(
                objectSampledPoints
            );

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

        cutPath.closed =
            curveClosed;

        cutPath.crossings = crossings;

        cutPath.cutVertices =
            CurveMeshIntersector::buildCutVertices(
                cutPath.crossings,
                duplicateTolerance
            );

        cutPath.faceIntervalIds =
            CurveMeshIntersector::deriveFaceIntervals(
                cutPath,
                mayaHalfEdgeMesh
            );

        std::vector<int> influencedFaceIds =
            CurveMeshIntersector::collectUniqueFaces(
                cutPath.faceIntervalIds
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

        ProfileCutInput profileInput;

        profileInput.curveId =
            static_cast<int>(
                curveIndex
            );

        profileInput.authoredStartNodeId =
            authoredNodeId(inputCurveStartNodeIds, curveIndex);
        profileInput.authoredEndNodeId =
            authoredNodeId(inputCurveEndNodeIds, curveIndex);

        profileInput.closed =
            curveClosed;

        profileInput.sampledSegments =
            sampledSegments;

        profileInputs.push_back(
            profileInput
        );

        debugCrossings.insert(
            debugCrossings.end(),
            crossings.begin(),
            crossings.end()
        );
    }

    const bool hasExplicitAuthoredTopology =
        !profileInputs.empty() &&
        std::all_of(
            profileInputs.begin(),
            profileInputs.end(),
            [](const ProfileCutInput& input)
            {
                return input.authoredStartNodeId >= 0 &&
                    input.authoredEndNodeId >= 0;
            }
        );

    if (hasExplicitAuthoredTopology)
    {
        struct AuthoredEndpoint
        {
            int curveId;
            CurveEndpoint endpoint;
        };

        std::unordered_map<int, std::vector<AuthoredEndpoint>>
            endpointsByNodeId;

        for (const ProfileCutInput& input : profileInputs)
        {
            endpointsByNodeId[input.authoredStartNodeId].push_back(
                {input.curveId, CurveEndpoint::Start}
            );
            endpointsByNodeId[input.authoredEndNodeId].push_back(
                {input.curveId, CurveEndpoint::End}
            );
        }

        for (const auto& entry : endpointsByNodeId)
        {
            const std::vector<AuthoredEndpoint>& endpoints = entry.second;

            if (endpoints.size() < 2)
            {
                continue;
            }

            const AuthoredEndpoint& target = endpoints.front();

            for (size_t endpointIndex = 1;
                 endpointIndex < endpoints.size();
                 ++endpointIndex)
            {
                const AuthoredEndpoint& source = endpoints[endpointIndex];
                ProfileCurveConnection connection;
                connection.endpoint = source.endpoint;
                connection.targetCurveId = target.curveId;
                connection.targetSegmentId =
                    target.endpoint == CurveEndpoint::Start
                        ? 0
                        : static_cast<int>(
                              profileInputs[target.curveId]
                                  .sampledSegments.size()
                          ) - 1;
                connection.targetSegmentT =
                    target.endpoint == CurveEndpoint::Start ? 0.0 : 1.0;
                profileInputs[source.curveId].connections.push_back(
                    connection
                );
            }
        }
    }

    /* Fall back to geometric discovery for legacy curve inputs. */
    const double connectionTolerance =
        std::max(
            0.001,
            meanMeshEdgeLength * 0.35
        );

    const std::vector<DetectedCurveConnection>
        detectedProfileConnections =
            hasExplicitAuthoredTopology
                ? std::vector<DetectedCurveConnection>{}
                : CurveConnectionDetector::detect(
                      currentSampledCurves,
                      connectionTolerance
                  );

    for (const DetectedCurveConnection& detected :
         detectedProfileConnections)
    {
        for (ProfileCutInput& profileInput :
             profileInputs)
        {
            if (profileInput.curveId !=
                detected.endpointCurveId)
            {
                continue;
            }

            ProfileCurveConnection connection;

            connection.endpoint =
                detected.endpoint;

            connection.targetCurveId =
                detected.targetCurveId;

            connection.targetSegmentId =
                detected.targetSegmentId;

            connection.targetSegmentT =
                detected.targetSegmentT;

            profileInput.connections.push_back(
                connection
            );

            break;
        }
    }

    std::stable_sort(
        profileInputs.begin(),
        profileInputs.end(),
        [](
            const ProfileCutInput& first,
            const ProfileCutInput& second
        )
        {
            return first.connections.size() <
                   second.connections.size();
        }
    );

    if (!cutPaths.empty())
    {
        const int originalVertexCount =
            static_cast<int>(
                mayaHalfEdgeMesh.vertices.size()
            );

        const int originalHalfEdgeCount =
            static_cast<int>(
                mayaHalfEdgeMesh.halfEdges.size()
            );

        const int originalFaceCount =
            static_cast<int>(
                mayaHalfEdgeMesh.faces.size()
            );

        const double crossingTolerance =
            0.01;

        const double duplicateTolerance =
            0.0001;

        CurvenetCutResult curvenetCutResult =
            CurvenetMeshCutter::apply(
                mayaHalfEdgeMesh,
                profileInputs,
                crossingTolerance,
                duplicateTolerance
            );


        /* Cache the exact crossings used by the cutter, not the preliminary
           proximity detections produced before the evolving-mesh pass. */
        debugCrossings.clear();

        for (const CutPath& attemptedPath :
             curvenetCutResult.attemptedCutPaths)
        {
            for (const CutCrossing& objectCrossing :
                 attemptedPath.crossings)
            {
                CutCrossing worldCrossing = objectCrossing;
                const MPoint worldPosition = MPoint(
                    objectCrossing.position.x,
                    objectCrossing.position.y,
                    objectCrossing.position.z
                ) * geometryLocalToWorldMatrix;
                worldCrossing.position = {
                    worldPosition.x,
                    worldPosition.y,
                    worldPosition.z
                };
                debugCrossings.push_back(worldCrossing);
            }
        }

        MGlobal::displayInfo(
            MString("Curvenet cutting: ")
            + (curvenetCutResult.success
                ? "SUCCESS"
                : "FAILED")
        );

        if (curvenetCutResult.failedCurveId >= 0)
        {
            MString failureName =
                "Unknown";
            int failedCrossingCount = -1;
            int failedCutVertexCount = -1;
            int failedFaceIntervalCount = -1;

            for (const CutPath& attemptedCutPath :
                 curvenetCutResult.attemptedCutPaths)
            {
                if (attemptedCutPath.curveId ==
                    curvenetCutResult.failedCurveId)
                {
                    failedCrossingCount = static_cast<int>(
                        attemptedCutPath.crossings.size()
                    );
                    failedCutVertexCount = static_cast<int>(
                        attemptedCutPath.cutVertices.size()
                    );
                    failedFaceIntervalCount = static_cast<int>(
                        attemptedCutPath.faceIntervalIds.size()
                    );
                    break;
                }
            }

            switch (
                curvenetCutResult.failedSplitReason
            )
            {
                case CutPathSplitFailure::None:
                    failureName = "None";
                    break;

                case CutPathSplitFailure::NullCutVertex:
                    failureName = "NullCutVertex";
                    break;

                case CutPathSplitFailure::
                    InvalidExistingMeshVertex:
                    failureName =
                        "InvalidExistingMeshVertex";
                    break;

                case CutPathSplitFailure::InvalidHalfEdge:
                    failureName = "InvalidHalfEdge";
                    break;

                case CutPathSplitFailure::
                    BoundarySplitFailed:
                    failureName = "BoundarySplitFailed";
                    break;

                case CutPathSplitFailure::
                    InternalSplitFailed:
                    failureName = "InternalSplitFailed";
                    break;

                case CutPathSplitFailure::InvalidMeshVertex:
                    failureName = "InvalidMeshVertex";
                    break;

                case CutPathSplitFailure::
                    InvalidFaceInterval:
                    failureName = "InvalidFaceInterval";
                    break;

                case CutPathSplitFailure::
                    VerticesNotOnSameFace:
                    failureName = "VerticesNotOnSameFace";
                    break;

                case CutPathSplitFailure::
                    CreateCutHalfEdgesFailed:
                    failureName =
                        "CreateCutHalfEdgesFailed";
                    break;

                case CutPathSplitFailure::
                    InsertCutHalfEdgesFailed:
                    failureName =
                        "InsertCutHalfEdgesFailed";
                    break;

                case CutPathSplitFailure::
                    InvalidClosingHalfEdge:
                    failureName =
                        "InvalidClosingHalfEdge";
                    break;

                case CutPathSplitFailure::
                    ClosingEdgeMismatch:
                    failureName = "ClosingEdgeMismatch";
                    break;
                case CutPathSplitFailure::
                    IncompleteSurfaceTracking:
                    failureName = "IncompleteSurfaceTracking";
                    break;
            }

            MGlobal::displayInfo(
                MString("Fresh CutPath failed for curve ID: ")
                + curvenetCutResult.failedCurveId
                + ", reason: "
                + failureName
                + ", interval: "
                + curvenetCutResult.failedIntervalIndex
                + ", vertices: "
                + curvenetCutResult.failedFirstVertexId
                + " -> "
                + curvenetCutResult.failedSecondVertexId
                + ", crossings: "
                + failedCrossingCount
                + ", cut vertices: "
                + failedCutVertexCount
                + ", face intervals: "
                + failedFaceIntervalCount
            );
        }

        MGlobal::displayInfo(
            MString("Shared Curvenet nodes: ")
            + static_cast<int>(
                curvenetCutResult
                    .sharedCurvenetNodes
                    .size()
            )
        );

        int physicalNodeCount = 0;

        for (const SharedCurvenetNode& node :
             curvenetCutResult.sharedCurvenetNodes)
        {
            if (node.meshVertexId >= 0)
            {
                ++physicalNodeCount;
            }
        }

        MGlobal::displayInfo(
            MString("Physical Curvenet nodes: ") + physicalNodeCount +
            "/" + static_cast<int>(
                curvenetCutResult.sharedCurvenetNodes.size()
            )
        );

        MString incompleteChains("CutChains without physical edges:");
        int incompleteChainCount = 0;

        for (const auto& entry :
             curvenetCutResult.cutChainsByCurveId)
        {
            if (entry.second.halfEdgeIds.empty())
            {
                incompleteChains += MString(" ") + entry.first;
                ++incompleteChainCount;
            }
        }

        if (incompleteChainCount > 0)
        {
            MGlobal::displayWarning(incompleteChains);
        }

        MGlobal::displayInfo(
            MString("Surface-tracked CutPaths: ") +
            curvenetCutResult.surfaceTrackedCurveCount + "/" +
            static_cast<int>(profileInputs.size())
        );

        if (!curvenetCutResult.surfaceTrackingFailures.empty())
        {
            MString details("Surface tracking fallback:");

            for (const SurfaceTrackingFailure& failure :
                 curvenetCutResult.surfaceTrackingFailures)
            {
                details += MString(" curve ") + failure.curveId +
                    " (crossings " + failure.crossingCount +
                    ", intervals " + failure.intervalCount +
                    ", invalid " + failure.invalidIntervalCount + ")";
            }

            MGlobal::displayWarning(details);
        }

        if (curvenetCutResult.success)
        {
            const bool buildFullSurface =
                dataBlock.inputValue(
                    fullSurfaceCurvenet,
                    &status
                ).asBool();

            int expectedFullSurfaceFaceCount = -1;

            if (buildFullSurface && hasExplicitAuthoredTopology)
            {
                int meshEdgeCount = 0;

                for (int halfEdgeId = 0;
                     halfEdgeId < static_cast<int>(
                        mayaHalfEdgeMesh.halfEdges.size()
                     );
                     ++halfEdgeId)
                {
                    const int twinId =
                        mayaHalfEdgeMesh.halfEdges[halfEdgeId].twin;

                    if (twinId < 0 || halfEdgeId < twinId)
                    {
                        ++meshEdgeCount;
                    }
                }

                const int surfaceEulerCharacteristic =
                    static_cast<int>(mayaHalfEdgeMesh.vertices.size()) -
                    meshEdgeCount +
                    static_cast<int>(mayaHalfEdgeMesh.faces.size());
                const int cycleRank =
                    authoredCycleRank(profileInputs);
                const int cappedSurfaceEulerCharacteristic =
                    surfaceEulerCharacteristic +
                    meshBoundaryComponentCount(mayaHalfEdgeMesh);

                if (cycleRank >= 0)
                {
                    expectedFullSurfaceFaceCount =
                        cycleRank +
                        cappedSurfaceEulerCharacteristic - 1;
                }
            }

            if (buildFullSurface)
            {
                MGlobal::displayInfo("Curvenet coverage: FULL SURFACE");
                CurvenetFaceRegionBuilder::
                    buildFullSurfacePartitions(
                        curvenetCutResult,
                        expectedFullSurfaceFaceCount
                    );
            }
            else
            {
                MGlobal::displayInfo("Curvenet coverage: AUTHORED FACES");
                const int expectedAuthoredFaceCount =
                    authoredCycleRank(profileInputs);
                CurvenetFaceRegionBuilder::
                    buildAuthoredSurfacePartitions(
                        curvenetCutResult,
                        expectedAuthoredFaceCount
                    );

                if (expectedAuthoredFaceCount >= 0 &&
                    static_cast<int>(
                        curvenetCutResult.curvenetFaces.size()
                    ) != expectedAuthoredFaceCount)
                {
                    MGlobal::displayError(
                        MString("Curvenet topology validation failed: ") +
                        "expected " + expectedAuthoredFaceCount +
                        " authored faces, actual " +
                        static_cast<int>(
                            curvenetCutResult.curvenetFaces.size()
                        )
                    );
                }

            }

            CurvenetEdgeBuilder::build(
                curvenetCutResult,
                profileInputs
            );

            harmonicSolver.initialize(
                curvenetCutResult.mesh,
                curvenetCutResult.cutChainsByCurveId,
                originalVertexCount,
                neutralSampledCurves
            );

            if (buildFullSurface && hasExplicitAuthoredTopology)
            {
                const int expectedFaceCount =
                    expectedFullSurfaceFaceCount;
                const int actualFaceCount = static_cast<int>(
                    curvenetCutResult.curvenetFaces.size()
                );

                if (actualFaceCount != expectedFaceCount)
                {
                    int smallestRegionPolygonCount = -1;
                    double smallestRegionArea =
                        std::numeric_limits<double>::infinity();

                    for (const CurvenetFace& face :
                         curvenetCutResult.curvenetFaces)
                    {
                        double regionArea = 0.0;

                        for (int meshFaceId : face.meshFaceIds)
                        {
                            const std::vector<int> halfEdgeIds =
                                curvenetCutResult.mesh.traverseFace(meshFaceId);

                            if (halfEdgeIds.size() < 3)
                            {
                                continue;
                            }

                            const Point3& anchor =
                                curvenetCutResult.mesh.vertices[
                                    curvenetCutResult.mesh.halfEdges[
                                        halfEdgeIds[0]
                                    ].startVertex
                                ].position;

                            for (int index = 1;
                                 index + 1 < static_cast<int>(halfEdgeIds.size());
                                 ++index)
                            {
                                const Point3& first =
                                    curvenetCutResult.mesh.vertices[
                                        curvenetCutResult.mesh.halfEdges[
                                            halfEdgeIds[index]
                                        ].startVertex
                                    ].position;
                                const Point3& second =
                                    curvenetCutResult.mesh.vertices[
                                        curvenetCutResult.mesh.halfEdges[
                                            halfEdgeIds[index + 1]
                                        ].startVertex
                                    ].position;
                                const double ax = first.x - anchor.x;
                                const double ay = first.y - anchor.y;
                                const double az = first.z - anchor.z;
                                const double bx = second.x - anchor.x;
                                const double by = second.y - anchor.y;
                                const double bz = second.z - anchor.z;
                                const double cx = ay * bz - az * by;
                                const double cy = az * bx - ax * bz;
                                const double cz = ax * by - ay * bx;
                                regionArea += 0.5 * std::sqrt(
                                    cx * cx + cy * cy + cz * cz
                                );
                            }
                        }

                        if (regionArea < smallestRegionArea)
                        {
                            smallestRegionArea = regionArea;
                            smallestRegionPolygonCount = static_cast<int>(
                                face.meshFaceIds.size()
                            );
                        }
                    }

                    MString cleanupDetails =
                        MString(", regions before cleanup: ") +
                        curvenetCutResult
                            .fullSurfaceRegionCountBeforeCleanup +
                        ", merged regions: " +
                        static_cast<int>(
                            curvenetCutResult
                                .mergedFullSurfaceRegionAreas
                                .size()
                        );

                    for (int mergedIndex = 0;
                         mergedIndex < static_cast<int>(
                            curvenetCutResult
                                .mergedFullSurfaceRegionAreas
                                .size()
                         );
                         ++mergedIndex)
                    {
                        cleanupDetails +=
                            MString(" [") +
                            curvenetCutResult
                                .mergedFullSurfaceRegionPolygonCounts[
                                    mergedIndex
                                ] +
                            " polygons, area " +
                            curvenetCutResult
                                .mergedFullSurfaceRegionAreas[
                                    mergedIndex
                                ] +
                            "]";
                    }

                    MGlobal::displayError(
                        MString("Curvenet topology validation failed: ") +
                        "expected " + expectedFaceCount +
                        " faces, actual " + actualFaceCount +
                        ", smallest region: " +
                        smallestRegionPolygonCount + " polygons, area " +
                        smallestRegionArea +
                        cleanupDetails
                    );
                }
            }

            MGlobal::displayInfo(
                MString("Curvenet faces: ")
                + static_cast<int>(
                    curvenetCutResult
                        .curvenetFaces
                        .size()
                )
            );

            int mappedMeshFaceCount = 0;

            for (const CurvenetFace& curvenetFace :
                 curvenetCutResult.curvenetFaces)
            {
                mappedMeshFaceCount +=
                    static_cast<int>(
                        curvenetFace
                            .meshFaceIds
                            .size()
                    );
            }

            MGlobal::displayInfo(
                MString("Mapped mesh faces: ")
                + mappedMeshFaceCount
            );

            MGlobal::displayInfo(
                MString("Unmapped mesh faces: ") +
                std::max(
                    0,
                    static_cast<int>(curvenetCutResult.mesh.faces.size()) -
                        mappedMeshFaceCount
                )
            );

            topologyCaptured = true;

        }
    }

    if (!vertexBindingsCaptured)
    {
        vertexBindingsCaptured = true;

    }

    if (!neutralSamplesCaptured)
    {
        neutralSamplesCaptured = true;
    }

    geoIterator.reset();

    const std::vector<Point3> harmonicDisplacements =
        harmonicSolver.solve(currentSampledCurves);

    while (!geoIterator.isDone())
    {
        const int vertexId =
            static_cast<int>(geoIterator.index());

        if (vertexId >= 0 &&
            vertexId < static_cast<int>(harmonicDisplacements.size()))
        {
            MPoint vertexPosition = geoIterator.position();
            vertexPosition.x += harmonicDisplacements[vertexId].x;
            vertexPosition.y += harmonicDisplacements[vertexId].y;
            vertexPosition.z += harmonicDisplacements[vertexId].z;
            geoIterator.setPosition(vertexPosition);
            geoIterator.next();
            continue;
        }

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

const std::vector<CurveConnection>&
CurveDeformerNode::getDebugConnections() const
{
    return curvenetData.getConnections();
}

const std::vector<ProfileCurveData>&
CurveDeformerNode::getDebugProfileCurves() const
{
    return curvenetData.getCurves();
}

MTypeId CurveDeformerNode::id(0x001226C1);
MString CurveDeformerNode::nodeName("curvenetNode");
MObject CurveDeformerNode::inputCurves;
MObject CurveDeformerNode::inputCurveStartNodeIds;
MObject CurveDeformerNode::inputCurveEndNodeIds;
MObject CurveDeformerNode::inputDriverCurve;
MObject CurveDeformerNode::inputDriverNodeIds;
MObject CurveDeformerNode::inputMesh;
MObject CurveDeformerNode::fullSurfaceCurvenet;
MObject CurveDeformerNode::showGeneratedCurvenet;

MStatus initializePlugin(MObject pluginObject)
{
    MStatus status;
    MFnPlugin plugin(
        pluginObject,
        "Osher",
        "1.4-stable-local-curve-data",
        "Any"
    );

    MGlobal::displayInfo(
        "Curvenet plugin build: 1.4-stable-local-curve-data"
    );

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
