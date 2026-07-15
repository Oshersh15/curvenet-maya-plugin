#include "CurvenetDebugCommand.h"

#include "CurvenetNode.h"

#include <maya/MFnDependencyNode.h>
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

    const std::vector<std::vector<Point3>>&
        sampledCurves =
            curvenetNode->getDebugSampledCurves();

    const std::vector<CutCrossing>& crossings =
        curvenetNode->getDebugCrossings();

    const std::vector<CurveConnection>& connections =
        curvenetNode->getDebugConnections();

    if (sampledCurves.empty() &&
        crossings.empty() &&
        connections.empty())
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

    for (int crossingIndex = 0;
         crossingIndex < static_cast<int>(crossings.size());
         ++crossingIndex)
    {
        const CutCrossing& crossing =
            crossings[crossingIndex];

        MString locatorName =
            MString("curvenetDebug_crossing_")
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
            + "Shape.localScaleX\" 0.08; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleY\" 0.08; "
            + "setAttr \""
            + locatorName
            + "Shape.localScaleZ\" 0.08;";

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
