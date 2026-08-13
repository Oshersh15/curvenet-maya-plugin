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
#include <maya/MPlug.h>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "HalfEdge.h"
#include "MayaMeshConverter.h"
#include "ProfileCurveSampler.h"
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
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
#include <memory>

namespace
{
    bool parseProfileCoordinates(
        const MString& serializedCoordinates,
        std::vector<Point3>& points
    )
    {
        points.clear();
        const char* cursor = serializedCoordinates.asChar();
        std::vector<double> coordinates;

        while (cursor != nullptr && *cursor != '\0')
        {
            char* end = nullptr;
            const double coordinate = std::strtod(cursor, &end);

            if (end == cursor || !std::isfinite(coordinate))
            {
                return false;
            }

            coordinates.push_back(coordinate);
            cursor = end;

            if (*cursor == ',')
            {
                ++cursor;
            }
            else if (*cursor != '\0')
            {
                return false;
            }
        }

        if (coordinates.size() < 6 || coordinates.size() % 3 != 0)
        {
            return false;
        }

        points.reserve(coordinates.size() / 3);

        for (size_t index = 0; index < coordinates.size(); index += 3)
        {
            points.push_back(Point3{
                coordinates[index],
                coordinates[index + 1],
                coordinates[index + 2]
            });
        }

        return true;
    }

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

    inputCurveCoordinates = typedAttr.create(
        "inputCurveCoordinates",
        "icc",
        MFnData::kString,
        &status
    );

    if (!status)
    {
        status.perror("Failed to create inputCurveCoordinates attribute");
        return status;
    }

    typedAttr.setStorable(true);
    typedAttr.setReadable(false);
    typedAttr.setWritable(true);
    typedAttr.setConnectable(false);
    typedAttr.setArray(true);
    typedAttr.setUsesArrayDataBuilder(true);
    status = addAttribute(inputCurveCoordinates);

    if (!status)
    {
        status.perror("Failed to add inputCurveCoordinates attribute");
        return status;
    }

    attributeAffects(inputCurveCoordinates, outputGeom);

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
    /* Embedding construction is an explicit setup operation. Keeping it out
       of DG evaluation prevents Maya from recursively evaluating partially
       populated curve inputs. */
    if (!topologyCaptured ||
        !MPlug(thisMObject(), inputDriverCurve).isConnected())
    {
        return MS::kSuccess;
    }

    return evaluatePreparedState(
        &dataBlock, &geoIterator, localToWorldMatrix, geometryIndex
    );
}

MStatus CurveDeformerNode::prepareEmbedding(
    const MDagPath& meshPath,
    const std::vector<std::vector<Point3>>& profilePoints,
    const std::vector<int>& startNodeIds,
    const std::vector<int>& endNodeIds,
    bool fullSurface
)
{
    topologyCaptured = false;
    neutralSamplesCaptured = false;
    neutralDriverCaptured = false;
    vertexBindingsCaptured = false;
    neutralSampledCurves.clear();
    currentSampledCurves.clear();
    neutralDriverPositions.clear();
    neutralDriverFrames.clear();
    vertexBindings.clear();
    preparedInfoMessages.clear();
    preparedWarningMessages.clear();
    preparedErrorMessages.clear();

    return evaluatePreparedState(
        nullptr,
        nullptr,
        meshPath.inclusiveMatrix(),
        0,
        &meshPath,
        &profilePoints,
        &startNodeIds,
        &endNodeIds,
        &fullSurface
    );
}

void CurveDeformerNode::installPreparedEmbedding(
    CurveDeformerNode&& preparedNode
)
{
    /* Only completed, Maya-independent data crosses onto the live node. The
       atomic evaluation guard belongs to each node and is intentionally not
       transferred. */
    curvenetData = std::move(preparedNode.curvenetData);
    debugSampledCurves = std::move(preparedNode.debugSampledCurves);
    debugCrossings = std::move(preparedNode.debugCrossings);
    neutralSampledCurves = std::move(preparedNode.neutralSampledCurves);
    neutralSamplesCaptured = preparedNode.neutralSamplesCaptured;
    currentSampledCurves = std::move(preparedNode.currentSampledCurves);
    neutralDriverPositions = std::move(
        preparedNode.neutralDriverPositions
    );
    neutralDriverFrames = std::move(preparedNode.neutralDriverFrames);
    neutralDriverCaptured = preparedNode.neutralDriverCaptured;
    vertexBindings = std::move(preparedNode.vertexBindings);
    harmonicSolver = std::move(preparedNode.harmonicSolver);
    vertexBindingsCaptured = preparedNode.vertexBindingsCaptured;
    topologyCaptured = preparedNode.topologyCaptured;
    preparedInfoMessages = std::move(preparedNode.preparedInfoMessages);
    preparedWarningMessages = std::move(
        preparedNode.preparedWarningMessages
    );
    preparedErrorMessages = std::move(preparedNode.preparedErrorMessages);
}

void CurveDeformerNode::reportPreparedEmbedding() const
{
    for (const MString& message : preparedInfoMessages)
        MGlobal::displayInfo(message);
    for (const MString& message : preparedWarningMessages)
        MGlobal::displayWarning(message);
    for (const MString& message : preparedErrorMessages)
        MGlobal::displayError(message);
}

MStatus CurveDeformerNode::evaluatePreparedState(
MDataBlock* dataBlock,
    MItGeometry* geometryIterator,
    const MMatrix& localToWorldMatrix,
    unsigned int geometryIndex,
    const MDagPath* preparationMeshPath,
    const std::vector<std::vector<Point3>>* preparationProfilePoints,
    const std::vector<int>* preparationStartNodeIds,
    const std::vector<int>* preparationEndNodeIds,
    const bool* preparationFullSurface
)
{
    MStatus status;

    const auto traceStage = [](const char*) {};
    const bool preparingDetachedCache = preparationMeshPath != nullptr;
    const auto reportInfo = [this, preparingDetachedCache](const MString& message)
    {
        if (preparingDetachedCache)
            preparedInfoMessages.push_back(message);
        else
            MGlobal::displayInfo(message);
    };
    const auto reportWarning = [this, preparingDetachedCache](const MString& message)
    {
        if (preparingDetachedCache)
            preparedWarningMessages.push_back(message);
        else
            MGlobal::displayWarning(message);
    };
    const auto reportError = [this, preparingDetachedCache](const MString& message)
    {
        if (preparingDetachedCache)
            preparedErrorMessages.push_back(message);
        else
            MGlobal::displayError(message);
    };

    /* Maya may request this output recursively while resolving connected
       geometry. A nested evaluation must not mutate the outer evaluation's
       embedding caches. */
    if (deformInProgress.exchange(true))
    {
        traceStage("SKIP re-entrant deform");
        return MS::kSuccess;
    }

    struct DeformEvaluationGuard
    {
        std::atomic_bool& inProgress;

        ~DeformEvaluationGuard()
        {
            inProgress.store(false);
        }
    } evaluationGuard{deformInProgress};

    traceStage("before geometry filter");
    MMatrix geometryLocalToWorldMatrix =
        localToWorldMatrix;
    MString geometryTransformName;

    if (preparationMeshPath != nullptr)
    {
        geometryLocalToWorldMatrix = preparationMeshPath->inclusiveMatrix();
        MDagPath transformPath = *preparationMeshPath;
        if (!transformPath.hasFn(MFn::kTransform))
        {
            transformPath.pop();
        }
        geometryTransformName = transformPath.fullPathName();
    }
    else
    {
        MFnGeometryFilter geometryFilter(thisMObject(), &status);
        traceStage("after geometry filter");
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
        traceStage("before neutral mesh handle");
        if (preparationMeshPath != nullptr)
        {
            MFnMesh meshFn(*preparationMeshPath, &status);
            if (status)
            {
                traceStage("before MFnMesh");
                traceStage("before half-edge conversion");

                status = MayaMeshConverter::buildFromMayaMesh(
                    meshFn,
                    mayaHalfEdgeMesh
                );

                if (!status)
                {
                    reportError(
                        "Curvenet could not snapshot the input mesh topology."
                    );
                    return status;
                }

                meanMeshEdgeLength =
                    mayaHalfEdgeMesh.computeMeanEdgeLength();
                traceStage("after half-edge conversion");
            }
        }
        else
        {
            return MS::kFailure;
        }
    }

    traceStage("before profile snapshot");
    std::vector<std::vector<Point3>> inputProfilePoints;
    std::vector<int> authoredStartNodeIds;
    std::vector<int> authoredEndNodeIds;

    const auto snapshotNodeIds =
        [this](const MObject& attribute, std::vector<int>& values)
        {
            MPlug plug(thisMObject(), attribute);
            for (unsigned int index = 0; index < plug.numElements(); ++index)
            {
                MPlug element = plug.elementByPhysicalIndex(index);
                const unsigned int logicalIndex = element.logicalIndex();
                if (values.size() <= logicalIndex)
                    values.resize(logicalIndex + 1, -1);
                values[logicalIndex] = element.asInt();
            }
        };

    unsigned int numConnectedCurves = 0;

    if (preparationProfilePoints != nullptr &&
        preparationStartNodeIds != nullptr &&
        preparationEndNodeIds != nullptr)
    {
        inputProfilePoints = *preparationProfilePoints;
        authoredStartNodeIds = *preparationStartNodeIds;
        authoredEndNodeIds = *preparationEndNodeIds;
        numConnectedCurves = static_cast<unsigned int>(
            inputProfilePoints.size()
        );
    }
    else
    {
        snapshotNodeIds(inputCurveStartNodeIds, authoredStartNodeIds);
        snapshotNodeIds(inputCurveEndNodeIds, authoredEndNodeIds);

        MPlug coordinatePlug(thisMObject(), inputCurveCoordinates);
        for (unsigned int physicalIndex = 0;
             physicalIndex < coordinatePlug.numElements(); ++physicalIndex)
        {
            MPlug element = coordinatePlug.elementByPhysicalIndex(physicalIndex);
            numConnectedCurves = std::max(
                numConnectedCurves, element.logicalIndex() + 1
            );
        }
        inputProfilePoints.resize(numConnectedCurves);
        for (unsigned int physicalIndex = 0;
             physicalIndex < coordinatePlug.numElements(); ++physicalIndex)
        {
            MPlug element = coordinatePlug.elementByPhysicalIndex(physicalIndex);
            parseProfileCoordinates(
                element.asString(), inputProfilePoints[element.logicalIndex()]
            );
        }
    }

    const auto authoredNodeId = [
        &authoredStartNodeIds,
        &authoredEndNodeIds
    ](bool start, unsigned int index)
    {
        const std::vector<int>& values =
            start ? authoredStartNodeIds : authoredEndNodeIds;
        return index < values.size() ? values[index] : -1;
    };

    traceStage("after profile snapshot");

    std::vector<CutPath> cutPaths;
    std::vector<ProfileCutInput> profileInputs;

    std::unordered_map<int, Point3> currentDriverPositions;
    std::unordered_map<int, std::vector<Point3>> currentDriverFramePoints;
    const bool hasConnectedDriver = topologyCaptured &&
        preparationMeshPath == nullptr &&
        MPlug(thisMObject(), inputDriverCurve).isConnected();
    traceStage("after driver guard");

    if (hasConnectedDriver && dataBlock != nullptr)
    {
        MDataHandle driverHandle =
            dataBlock->inputValue(inputDriverCurve, &status);

        if (!status)
        {
            return MS::kSuccess;
        }

        const MObject driverObject =
            driverHandle.asNurbsCurve();

        if (!driverObject.isNull())
        {
            MFnNurbsCurve driverFn(driverObject, &status);

            if (status)
            {
                MArrayDataHandle driverIdHandle =
                    dataBlock->inputArrayValue(inputDriverNodeIds, &status);
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
            authoredNodeId(true, curveIndex);
        const int endNodeId =
            authoredNodeId(false, curveIndex);

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
            authoredNodeId(true, curveIndex);
        const int endLogicalNodeId =
            authoredNodeId(false, curveIndex);
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
        traceStage("profile begin");
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

        const bool curveClosed = false;

        if (curveIndex >= inputProfilePoints.size() ||
            inputProfilePoints[curveIndex].size() < 2)
        {
            traceStage("invalid profile coordinates");
            continue;
        }

        const std::vector<Point3>& controlPoints =
            inputProfilePoints[curveIndex];
        std::vector<Point3> objectSampledPoints = controlPoints;

        applyDriverDisplacement(curveIndex, objectSampledPoints);

        currentSampledCurves.emplace_back(objectSampledPoints);

        if (!neutralSamplesCaptured)
        {
            neutralSampledCurves.emplace_back(objectSampledPoints);
        }

        debugSampledCurves.emplace_back(controlPoints);

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
        traceStage("profile segments ready");

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
        traceStage("profile crossings ready");


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
            authoredNodeId(true, curveIndex);
        profileInput.authoredEndNodeId =
            authoredNodeId(false, curveIndex);

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
        traceStage("before cutter");
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
        traceStage("after cutter");


        /* Cache the exact crossings used by the cutter, not the preliminary
           proximity detections produced before the evolving-mesh pass. */
        debugCrossings.clear();
        traceStage("before debug crossing snapshot");

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
        traceStage("after debug crossing snapshot");

        reportInfo(
            MString("Curvenet cutting: ")
            + (curvenetCutResult.success
                ? "SUCCESS"
                : "FAILED")
        );

        if (!curvenetCutResult.success &&
            curvenetCutResult.failedCurveId >= 0)
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

            reportInfo(
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

        reportInfo(
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

        reportInfo(
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
            reportWarning(incompleteChains);
        }

        reportInfo(
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

            reportWarning(details);
        }

        if (curvenetCutResult.success)
        {
            traceStage("successful cut processing begin");
            const bool buildFullSurface =
                preparationFullSurface != nullptr
                    ? *preparationFullSurface
                    : MPlug(thisMObject(), fullSurfaceCurvenet).asBool();

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
                traceStage("before full-surface face regions");
                reportInfo("Curvenet coverage: FULL SURFACE");
                CurvenetFaceRegionBuilder::
                    buildFullSurfacePartitions(
                        curvenetCutResult,
                        expectedFullSurfaceFaceCount
                    );
                traceStage("after full-surface face regions");
            }
            else
            {
                traceStage("before authored face regions");
                reportInfo("Curvenet coverage: AUTHORED FACES");
                const int expectedAuthoredFaceCount =
                    authoredCycleRank(profileInputs);
                CurvenetFaceRegionBuilder::
                    buildAuthoredSurfacePartitions(
                        curvenetCutResult,
                        expectedAuthoredFaceCount
                    );
                traceStage("after authored face regions");

                if (expectedAuthoredFaceCount >= 0 &&
                    static_cast<int>(
                        curvenetCutResult.curvenetFaces.size()
                    ) != expectedAuthoredFaceCount)
                {
                    reportError(
                        MString("Curvenet topology validation failed: ") +
                        "expected " + expectedAuthoredFaceCount +
                        " authored faces, actual " +
                        static_cast<int>(
                            curvenetCutResult.curvenetFaces.size()
                        )
                    );
                }

            }

            traceStage("before Curvenet edge builder");
            CurvenetEdgeBuilder::build(
                curvenetCutResult,
                profileInputs
            );
            traceStage("after Curvenet edge builder");

            traceStage("before harmonic initialize");
            harmonicSolver.initialize(
                curvenetCutResult.mesh,
                curvenetCutResult.cutChainsByCurveId,
                originalVertexCount,
                neutralSampledCurves
            );
            traceStage("after harmonic initialize");

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

                    reportError(
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

            reportInfo(
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

            reportInfo(
                MString("Mapped mesh faces: ")
                + mappedMeshFaceCount
            );

            reportInfo(
                MString("Unmapped mesh faces: ") +
                std::max(
                    0,
                    static_cast<int>(curvenetCutResult.mesh.faces.size()) -
                        mappedMeshFaceCount
                )
            );

            topologyCaptured = true;
            traceStage("topology captured");

        }
    }

    if (!vertexBindingsCaptured && topologyCaptured)
    {
        vertexBindingsCaptured = true;

    }

    if (!neutralSamplesCaptured &&
        topologyCaptured &&
        !neutralSampledCurves.empty())
    {
        neutralSamplesCaptured = true;
    }

    if (geometryIterator == nullptr)
    {
        return topologyCaptured ? MS::kSuccess : MS::kFailure;
    }

    MItGeometry& geoIterator = *geometryIterator;
    geoIterator.reset();

    traceStage("before harmonic solve");
    const std::vector<Point3> harmonicDisplacements =
        harmonicSolver.solve(currentSampledCurves);
    traceStage("after harmonic solve");

    traceStage("before geometry update");
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

    traceStage("after geometry update");

    traceStage("EXIT deform");

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

namespace
{
class InitializeCurvenetEmbeddingCommand : public MPxCommand
{
public:
    static void* creator()
    {
        return new InitializeCurvenetEmbeddingCommand();
    }

    MStatus doIt(const MArgList& arguments) override
    {
        if (arguments.length() < 6 ||
            (arguments.length() - 3) % 3 != 0)
        {
            MGlobal::displayError(
                "initializeCurvenetEmbedding expects a deformer, mesh, "
                "coverage flag, "
                "and coordinate/start/end triples."
            );
            return MS::kInvalidParameter;
        }

        MStatus status;
        MSelectionList nodeSelection;
        status = nodeSelection.add(arguments.asString(0));
        MObject nodeObject;

        if (!status || !nodeSelection.getDependNode(0, nodeObject))
        {
            MGlobal::displayError("Curvenet deformer was not found.");
            return MS::kInvalidParameter;
        }

        MFnDependencyNode dependencyNode(nodeObject, &status);
        auto* node = status
            ? dynamic_cast<CurveDeformerNode*>(dependencyNode.userNode())
            : nullptr;

        if (node == nullptr)
        {
            MGlobal::displayError("Selected node is not a Curvenet deformer.");
            return MS::kInvalidParameter;
        }

        MSelectionList meshSelection;
        status = meshSelection.add(arguments.asString(1));
        MDagPath meshPath;

        if (!status || !meshSelection.getDagPath(0, meshPath))
        {
            MGlobal::displayError("Curvenet mesh was not found.");
            return MS::kInvalidParameter;
        }

        if (meshPath.hasFn(MFn::kTransform))
        {
            status = meshPath.extendToShape();
        }

        if (!status || !meshPath.hasFn(MFn::kMesh))
        {
            MGlobal::displayError("Curvenet target is not a polygon mesh.");
            return MS::kInvalidParameter;
        }

        const bool fullSurface = arguments.asBool(2, &status);

        if (!status)
        {
            return status;
        }

        const unsigned int curveCount = (arguments.length() - 3) / 3;
        std::vector<std::vector<Point3>> profilePoints(curveCount);
        std::vector<int> startNodeIds(curveCount, -1);
        std::vector<int> endNodeIds(curveCount, -1);

        for (unsigned int curveIndex = 0; curveIndex < curveCount; ++curveIndex)
        {
            const unsigned int argumentIndex = 3 + curveIndex * 3;
            if (!parseProfileCoordinates(
                    arguments.asString(argumentIndex),
                    profilePoints[curveIndex]
                ))
            {
                MGlobal::displayError(
                    MString("Invalid Curvenet coordinates for curve ") +
                    static_cast<int>(curveIndex) + "."
                );
                return MS::kInvalidParameter;
            }
            startNodeIds[curveIndex] = arguments.asInt(argumentIndex + 1);
            endNodeIds[curveIndex] = arguments.asInt(argumentIndex + 2);
        }

        /* Do not run topology construction on the Maya-owned deformer. Maya
           can inspect that node while this command is executing. Build the
           complete cache on an ordinary C++ object, then install it in one
           short, non-computational step. */
        CurveDeformerNode preparedNode;
        status = preparedNode.prepareEmbedding(
            meshPath,
            profilePoints,
            startNodeIds,
            endNodeIds,
            fullSurface
        );

        if (!status)
        {
            MGlobal::displayError("Curvenet embedding preparation failed.");
            return status;
        }

        node->installPreparedEmbedding(std::move(preparedNode));
        node->reportPreparedEmbedding();
        return MS::kSuccess;
    }
};
}

MTypeId CurveDeformerNode::id(0x001226C1);
MString CurveDeformerNode::nodeName("curvenetNode");
MObject CurveDeformerNode::inputCurves;
MObject CurveDeformerNode::inputCurveCoordinates;
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
        "6.0-isolated-embedding-cache",
        "Any"
    );

    MGlobal::displayInfo(
        "Curvenet plugin build: 6.0-isolated-embedding-cache"
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

    status = plugin.registerCommand(
        "initializeCurvenetEmbedding",
        InitializeCurvenetEmbeddingCommand::creator
    );

    if (!status)
    {
        status.perror("Failed to register initializeCurvenetEmbedding command");
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

    status = plugin.deregisterCommand("initializeCurvenetEmbedding");

    if (!status)
    {
        status.perror("Failed to deregister initializeCurvenetEmbedding command");
        return status;
    }

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
