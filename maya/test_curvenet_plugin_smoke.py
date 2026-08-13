"""Headless Maya smoke test for the complete Curvenet deformer lifecycle."""

import os
import sys

import maya.standalone


standalone_initialized = False
try:
    maya.standalone.initialize(name="python")
    standalone_initialized = True
except RuntimeError:
    # Maya batch already owns the standalone application lifecycle.
    pass

import maya.cmds as cmds  # noqa: E402
import maya.api.OpenMaya as om  # noqa: E402


SCRIPT_PATH = globals().get(
    "__file__",
    os.path.join(os.getcwd(), "maya", "test_curvenet_plugin_smoke.py"),
)
PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(SCRIPT_PATH)))
EXTENSION = ".bundle" if sys.platform == "darwin" else ".so"
PLUGIN = os.environ.get(
    "CURVENET_PLUGIN_PATH",
    os.path.join(PROJECT, "plugin", "build", "CurvenetPlugin" + EXTENSION),
)

cmds.loadPlugin(PLUGIN)
mesh = cmds.polyPlane(
    name="curvenetSmokeMesh",
    width=2.0,
    height=2.0,
    subdivisionsX=4,
    subdivisionsY=4,
)[0]
curves = (
    ((-0.6, 0.0, -0.6), (0.6, 0.0, -0.6), 0, 1),
    ((0.6, 0.0, -0.6), (0.6, 0.0, 0.6), 1, 2),
    ((0.6, 0.0, 0.6), (-0.6, 0.0, 0.6), 2, 3),
    ((-0.6, 0.0, 0.6), (-0.6, 0.0, -0.6), 3, 0),
)
curve_inputs = []
preparation_arguments = []

for curve_id, (start, end, start_id, end_id) in enumerate(curves):
    points = (
        start,
        tuple((2.0 * start[i] + end[i]) / 3.0 for i in range(3)),
        tuple((start[i] + 2.0 * end[i]) / 3.0 for i in range(3)),
        end,
    )
    serialized = ",".join(str(value) for point in points for value in point)
    curve_inputs.append((serialized, start_id, end_id))
    preparation_arguments.extend((serialized, start_id, end_id))

deformer = cmds.deformer(
    mesh,
    type="curvenetNode",
    name="curvenetSmokeNode",
)[0]
cmds.setAttr(deformer + ".nodeState", 1)

for curve_id, (serialized, start_id, end_id) in enumerate(curve_inputs):
    cmds.setAttr(
        f"{deformer}.inputCurveCoordinates[{curve_id}]",
        serialized,
        type="string",
    )
    cmds.setAttr(f"{deformer}.inputCurveStartNodeIds[{curve_id}]", start_id)
    cmds.setAttr(f"{deformer}.inputCurveEndNodeIds[{curve_id}]", end_id)

cmds.initializeCurvenetEmbedding(
    deformer,
    mesh,
    False,
    *preparation_arguments,
)

driver_points = []
for logical_id, position in enumerate(
    (
        (-0.6, 0.0, -0.6),
        (0.6, 0.0, -0.6),
        (0.6, 0.0, 0.6),
        (-0.6, 0.0, 0.6),
    )
):
    x, y, z = position
    driver_points.extend(
        (
            (x, y, z),
            (x + 0.05, y, z),
            (x, y + 0.05, z),
            (x, y, z + 0.05),
        )
    )
    for frame_point in range(4):
        cmds.setAttr(
            f"{deformer}.inputDriverNodeIds[{logical_id * 4 + frame_point}]",
            logical_id,
        )

driver = cmds.curve(
    name="curvenetSmokeDriver",
    degree=1,
    point=driver_points,
)
driver_shape = cmds.listRelatives(driver, shapes=True, fullPath=True)[0]
cmds.connectAttr(driver_shape + ".local", deformer + ".inputDriverCurve")
cmds.setAttr(deformer + ".nodeState", 0)
cmds.dgdirty(deformer)
mesh_selection = om.MSelectionList()
mesh_selection.add(mesh)
mesh_path = mesh_selection.getDagPath(0)
mesh_path.extendToShape()
points = om.MFnMesh(mesh_path).getPoints(om.MSpace.kObject)
assert len(points) == 25
om.MGlobal.displayInfo("CURVENET_MAYA_SMOKE_TEST: PASS")

if standalone_initialized:
    maya.standalone.uninitialize()
