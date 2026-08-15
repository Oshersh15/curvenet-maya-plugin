#pragma once

/* Declares the Maya command that visualises cached embedding diagnostics. */

#include <maya/MPxCommand.h>
#include <maya/MStatus.h>
#include <maya/MString.h>

/*
    Maya command that creates viewport debug geometry from the
    sampled profile curves and CutCrossing data cached by a
    CurveDeformerNode.

    Run in Maya using:

        visualizeCurvenetDebug;
*/
class CurvenetDebugCommand : public MPxCommand
{
public:
    CurvenetDebugCommand() = default;
    ~CurvenetDebugCommand() override = default;

    static void* creator();

    MStatus doIt(const MArgList& arguments) override;

    static MString commandName;
};
