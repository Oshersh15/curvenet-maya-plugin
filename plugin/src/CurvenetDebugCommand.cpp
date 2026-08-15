/* Implements the Maya command used to inspect Curvenet embedding data. */

#include "CurvenetDebugCommand.h"

#include "CurvenetNode.h"

#include <maya/MFnDependencyNode.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MObject.h>
#include <maya/MPxNode.h>
#include <maya/MStringArray.h>

MString CurvenetDebugCommand::commandName(
    "visualizeCurvenetDebug"
);

void* CurvenetDebugCommand::creator()
{
    return new CurvenetDebugCommand();
}

MStatus CurvenetDebugCommand::doIt(
    const MArgList& arguments
)
{
    MStatus argumentStatus;
    const int requestedCurveId = arguments.length() > 0
        ? arguments.asInt(0, &argumentStatus)
        : -1;

    if (arguments.length() > 0 && !argumentStatus)
    {
        MGlobal::displayError(
            "visualizeCurvenetDebug expects an integer curve ID."
        );
        return MS::kFailure;
    }

    CurveDeformerNode* curvenetNode = nullptr;

    MItDependencyNodes nodeIterator(
        MFn::kPluginDeformerNode
    );

    for (;
         !nodeIterator.isDone();
         nodeIterator.next())
    {
        MObject nodeObject =
            nodeIterator.thisNode();

        MFnDependencyNode dependencyNode(
            nodeObject
        );

        if (dependencyNode.typeName() !=
            CurveDeformerNode::nodeName)
        {
            continue;
        }

        MPxNode* userNode =
            dependencyNode.userNode();

        curvenetNode =
            dynamic_cast<CurveDeformerNode*>(
                userNode
            );

        if (curvenetNode != nullptr)
        {
            break;
        }
    }

    if (curvenetNode == nullptr)
    {
        MGlobal::displayError(
            "No curvenetNode was found in the scene."
        );

        return MS::kFailure;
    }

    const std::vector<std::vector<Point3>> sampledCurves =
        curvenetNode->getDebugSampledCurves();

    const std::vector<CutCrossing> crossings =
        curvenetNode->getDebugCrossings();

    const std::vector<CurveConnection> connections;
    const std::vector<ProfileCurveData> profileCurves;

    if (sampledCurves.empty() &&
        crossings.empty() &&
        connections.empty() &&
        profileCurves.empty())
    {
        MGlobal::displayError(
            "The curvenetNode has no cached debug data. "
            "Make sure the deformer has evaluated first."
        );

        return MS::kFailure;
    }

    MStatus status;

    status = MGlobal::executeCommand(
        "if (`objExists \"curvenetDebug_group\"`) "
        "{ delete \"curvenetDebug_group\"; }",
        false,
        false
    );

    if (!status)
    {
        MGlobal::displayError(
            "Failed to delete the existing Curvenet debug group."
        );

        return status;
    }

    status = MGlobal::executeCommand(
        "group -empty -name \"curvenetDebug_group\";",
        false,
        false
    );

    if (!status)
    {
        MGlobal::displayError(
            "Failed to create the Curvenet debug group."
        );

        return status;
    }

    for (int curveIndex = 0;
         curveIndex < static_cast<int>(sampledCurves.size());
         ++curveIndex)
    {
        if (requestedCurveId >= 0 && curveIndex != requestedCurveId)
        {
            continue;
        }

        const std::vector<Point3>& sampledPoints =
            sampledCurves[curveIndex];

        if (sampledPoints.size() < 2)
        {
            continue;
        }

        MString curveName =
            MString("curvenetDebug_sampledCurve_")
            + curveIndex;

        MString curveCommand =
            MString("curve -degree 1 -name \"")
            + curveName
            + "\"";

        for (const Point3& point : sampledPoints)
        {
            curveCommand += " -point ";
            curveCommand += point.x;
            curveCommand += " ";
            curveCommand += point.y;
            curveCommand += " ";
            curveCommand += point.z;
        }

        curveCommand += ";";

        status = MGlobal::executeCommand(
            curveCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to create debug curve ")
                + curveIndex
            );

            return status;
        }

        MString parentCommand =
            MString("parent \"")
            + curveName
            + "\" \"curvenetDebug_group\";";

        status = MGlobal::executeCommand(
            parentCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to parent debug curve ")
                + curveIndex
            );

            return status;
        }

        MStringArray shapeNames;

        status = MGlobal::executeCommand(
            MString("listRelatives -shapes -fullPath \"")
            + curveName
            + "\";",
            shapeNames,
            false,
            false
        );

        if (!status || shapeNames.length() == 0)
        {
            MGlobal::displayError(
                MString("Failed to find the shape for debug curve ")
                + curveIndex
            );

            return MS::kFailure;
        }

        const MString shapeName =
            shapeNames[0];

        const int colourIndex =
            (curveIndex % 7) + 1;

        MString colourCommand =
            MString("setAttr \"")
            + shapeName
            + ".overrideEnabled\" 1; "
            + "setAttr \""
            + shapeName
            + ".overrideColor\" "
            + colourIndex
            + ";";

        status = MGlobal::executeCommand(
            colourCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to colour debug curve ")
                + curveIndex
            );

            return status;
        }
    }

    if (false)
    {
    std::vector<Point3> graphNodePositions;

    for (int curveIndex = 0;
         curveIndex < static_cast<int>(sampledCurves.size());
         ++curveIndex)
    {
        const std::vector<Point3>& sampledPoints =
            sampledCurves[curveIndex];

        if (sampledPoints.empty())
        {
            graphNodePositions.push_back(
                Point3{}
            );

            continue;
        }

        const Point3& middlePoint =
            sampledPoints[
                sampledPoints.size() / 2
            ];

        graphNodePositions.push_back(
            Point3{
                middlePoint.x + 4.0,
                middlePoint.y,
                middlePoint.z
            }
        );
    }

    for (int curveIndex = 0;
         curveIndex < static_cast<int>(graphNodePositions.size());
         ++curveIndex)
    {
        const Point3& graphPosition =
            graphNodePositions[curveIndex];

        MString locatorName =
            MString("curvenetDebug_graphNode_")
            + curveIndex;

        MString locatorCommand =
            MString("spaceLocator -name \"")
            + locatorName
            + "\" -position "
            + graphPosition.x
            + " "
            + graphPosition.y
            + " "
            + graphPosition.z
            + ";";

        status = MGlobal::executeCommand(
            locatorCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to create graph node ")
                + curveIndex
            );

            return status;
        }

        MString styleCommand =
            MString("setAttr \"")
            + locatorName
            + "Shape.localScaleX\" 0.25; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleY\" 0.25; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleZ\" 0.25; "
            + "setAttr \""
            + locatorName
            + "Shape.overrideEnabled\" 1; "
            + "setAttr \""
            + locatorName
            + "Shape.overrideColor\" 17;";

        status = MGlobal::executeCommand(
            styleCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to style graph node ")
                + curveIndex
            );

            return status;
        }

        MString parentCommand =
            MString("parent \"")
            + locatorName
            + "\" \"curvenetDebug_group\";";

        status = MGlobal::executeCommand(
            parentCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to parent graph node ")
                + curveIndex
            );

            return status;
        }
    }

    for (int connectionIndex = 0;
         connectionIndex < static_cast<int>(connections.size());
         ++connectionIndex)
    {
        const CurveConnection& connection =
            connections[connectionIndex];

        MGlobal::displayInfo(
            MString("Graph connection ")
            + connectionIndex
            + ": endpoint curve "
            + connection.endpointCurveId
            + ", target curve "
            + connection.targetCurveId
            + ", graph node count "
            + static_cast<int>(graphNodePositions.size())
        );

        if (connection.endpointCurveId < 0 ||
            connection.endpointCurveId >=
                static_cast<int>(graphNodePositions.size()) ||
            connection.targetCurveId < 0 ||
            connection.targetCurveId >=
                static_cast<int>(graphNodePositions.size()))
        {
            continue;
        }

        const Point3& startPosition =
            graphNodePositions[
                connection.endpointCurveId
            ];

        const Point3& endPosition =
            graphNodePositions[
                connection.targetCurveId
            ];

        MString curveName =
            MString("curvenetDebug_graphEdge_")
            + connectionIndex;

        MString curveCommand =
            MString("curve -degree 1 -name \"")
            + curveName
            + "\""
            + " -point "
            + startPosition.x
            + " "
            + startPosition.y
            + " "
            + startPosition.z
            + " -point "
            + endPosition.x
            + " "
            + endPosition.y
            + " "
            + endPosition.z
            + ";";

        status = MGlobal::executeCommand(
            curveCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to create graph edge ")
                + connectionIndex
            );

            return status;
        }

        MString parentCommand =
            MString("parent \"")
            + curveName
            + "\" \"curvenetDebug_group\";";

        status = MGlobal::executeCommand(
            parentCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to parent graph edge ")
                + connectionIndex
            );

            return status;
        }

        MStringArray shapeNames;

        status = MGlobal::executeCommand(
            MString("listRelatives -shapes -fullPath \"")
            + curveName
            + "\";",
            shapeNames,
            false,
            false
        );

        if (!status || shapeNames.length() == 0)
        {
            MGlobal::displayError(
                MString("Failed to find shape for graph edge ")
                + connectionIndex
            );

            return MS::kFailure;
        }

        const MString shapeName =
            shapeNames[0];

        MString styleCommand =
            MString("setAttr \"")
            + shapeName
            + ".overrideEnabled\" 1; "
            + "setAttr \""
            + shapeName
            + ".overrideColor\" 17; "
            + "setAttr \""
            + shapeName
            + ".lineWidth\" 4;";

        status = MGlobal::executeCommand(
            styleCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to style graph edge ")
                + connectionIndex
            );

            return status;
        }
    }

    }

    for (int curveIndex = 0;
         curveIndex < static_cast<int>(profileCurves.size());
         ++curveIndex)
    {
        const ProfileCurveData& profileCurve =
            profileCurves[curveIndex];

        struct EndpointMarker
        {
            MString label;
            MPoint position;
            int colourIndex;
        };

        const EndpointMarker endpointMarkers[2] =
        {
            {
                "start",
                profileCurve.startPoint,
                14
            },
            {
                "end",
                profileCurve.endPoint,
                13
            }
        };

        for (const EndpointMarker& marker :
             endpointMarkers)
        {
            MString locatorName =
                MString("curvenetDebug_curve_")
                + curveIndex
                + "_"
                + marker.label;

            MString locatorCommand =
                MString("spaceLocator -name \"")
                + locatorName
                + "\" -position "
                + marker.position.x
                + " "
                + marker.position.y
                + " "
                + marker.position.z
                + ";";

            status = MGlobal::executeCommand(
                locatorCommand,
                false,
                false
            );

            if (!status)
            {
                MGlobal::displayError(
                    MString("Failed to create ")
                    + marker.label
                    + " marker for curve "
                    + curveIndex
                );

                return status;
            }

            MString styleCommand =
                MString("setAttr \"")
                + locatorName
                + "Shape.localScaleX\" 0.12; "
                + "setAttr \""
                + locatorName
                + "Shape.localScaleY\" 0.12; "
                + "setAttr \""
                + locatorName
                + "Shape.localScaleZ\" 0.12; "
                + "setAttr \""
                + locatorName
                + "Shape.overrideEnabled\" 1; "
                + "setAttr \""
                + locatorName
                + "Shape.overrideColor\" "
                + marker.colourIndex
                + ";";

            status = MGlobal::executeCommand(
                styleCommand,
                false,
                false
            );

            if (!status)
            {
                MGlobal::displayError(
                    MString("Failed to style ")
                    + marker.label
                    + " marker for curve "
                    + curveIndex
                );

                return status;
            }

            MString parentCommand =
                MString("parent \"")
                + locatorName
                + "\" \"curvenetDebug_group\";";

            status = MGlobal::executeCommand(
                parentCommand,
                false,
                false
            );

            if (!status)
            {
                MGlobal::displayError(
                    MString("Failed to parent ")
                    + marker.label
                    + " marker for curve "
                    + curveIndex
                );

                return status;
            }
        }
    }

    for (int crossingIndex = 0;
         crossingIndex < static_cast<int>(crossings.size());
         ++crossingIndex)
    {
        const CutCrossing& crossing =
            crossings[crossingIndex];

        if (requestedCurveId >= 0 &&
            crossing.curveId != requestedCurveId)
        {
            continue;
        }

        MString locatorName =
            MString("curvenetDebug_curve_")
            + crossing.curveId
            + "_crossing_"
            + crossingIndex;

        MString locatorCommand =
            MString("spaceLocator -name \"")
            + locatorName
            + "\" -position "
            + crossing.position.x
            + " "
            + crossing.position.y
            + " "
            + crossing.position.z
            + ";";

        status = MGlobal::executeCommand(
            locatorCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to create crossing locator ")
                + crossingIndex
            );

            return status;
        }

        MString scaleCommand =
            MString("setAttr \"")
            + locatorName
            + "Shape.localScaleX\" 0.02; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleY\" 0.02; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleZ\" 0.02; "
            + "setAttr \""
            + locatorName
            + "Shape.overrideEnabled\" 1; "
            + "setAttr \""
            + locatorName
            + "Shape.overrideColor\" 13;";

        status = MGlobal::executeCommand(
            scaleCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to resize crossing locator ")
                + crossingIndex
            );

            return status;
        }

        MString parentCommand =
            MString("parent \"")
            + locatorName
            + "\" \"curvenetDebug_group\";";

        status = MGlobal::executeCommand(
            parentCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to parent crossing locator ")
                + crossingIndex
            );

            return status;
        }
    }

    for (int connectionIndex = 0;
         connectionIndex < static_cast<int>(connections.size());
         ++connectionIndex)
    {
        const CurveConnection& connection =
            connections[connectionIndex];

        MString locatorName =
            MString("curvenetDebug_connection_")
            + connectionIndex;

        MString locatorCommand =
            MString("spaceLocator -name \"")
            + locatorName
            + "\" -position "
            + connection.position.x
            + " "
            + connection.position.y
            + " "
            + connection.position.z
            + ";";

        status = MGlobal::executeCommand(
            locatorCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to create connection locator ")
                + connectionIndex
            );

            return status;
        }

        MString scaleCommand =
            MString("setAttr \"")
            + locatorName
            + "Shape.localScaleX\" 0.16; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleY\" 0.16; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleZ\" 0.16; "
            + "setAttr \""
            + locatorName
            + "Shape.overrideEnabled\" 1; "
            + "setAttr \""
            + locatorName
            + "Shape.overrideColor\" 17;";

        status = MGlobal::executeCommand(
            scaleCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to style connection locator ")
                + connectionIndex
            );

            return status;
        }

        MString parentCommand =
            MString("parent \"")
            + locatorName
            + "\" \"curvenetDebug_group\";";

        status = MGlobal::executeCommand(
            parentCommand,
            false,
            false
        );

        if (!status)
        {
            MGlobal::displayError(
                MString("Failed to parent connection locator ")
                + connectionIndex
            );

            return status;
        }
    }

    MGlobal::displayInfo(
        MString("Created Curvenet debug visualiser with ")
        + static_cast<int>(sampledCurves.size())
        + " sampled curve(s), "
        + static_cast<int>(crossings.size())
        + " crossing marker(s), and "
        + static_cast<int>(connections.size())
        + " connection marker(s)."
    );

    return MS::kSuccess;
}
