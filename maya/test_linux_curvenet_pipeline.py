"""Headless Linux regression test for the complete Curvenet Maya pipeline."""

import os
import math
import sys
import traceback

import maya.standalone


def _fail(message):
    print("CURVENET_LINUX_PIPELINE: FAIL: " + message, flush=True)
    return 1


def main():
    maya.standalone.initialize(name="python")

    import maya.cmds as cmds

    plugin_path = os.environ.get("CURVENET_PLUGIN_PATH")
    if not plugin_path or not os.path.isfile(plugin_path):
        return _fail("CURVENET_PLUGIN_PATH does not name a built plugin")

    cmds.loadPlugin(plugin_path)
    print(
        "CURVENET_LINUX_PLUGIN: "
        + cmds.pluginInfo(plugin_path, query=True, version=True),
        flush=True,
    )

    mesh = cmds.polyCylinder(
        name="linuxCurvenetTube",
        radius=1.5,
        height=5.0,
        subdivisionsAxis=24,
        subdivisionsHeight=18,
        subdivisionsCaps=1,
        createUVs=3,
    )[0]
    cmds.delete(mesh, constructionHistory=True)

    # Four sampled surface polylines forming one authored rectangular face.
    radius = 1.5
    left_angle = math.asin(-0.75 / radius)
    right_angle = math.asin(0.75 / radius)

    def horizontal(y_value, reverse=False):
        angles = [
            left_angle + (right_angle - left_angle) * index / 8.0
            for index in range(9)
        ]
        if reverse:
            angles.reverse()
        return tuple(
            (radius * math.sin(angle), y_value, radius * math.cos(angle))
            for angle in angles
        )

    def vertical(x_value, start_y, end_y):
        z_value = math.sqrt(radius * radius - x_value * x_value)
        return tuple(
            (
                x_value,
                start_y + (end_y - start_y) * index / 8.0,
                z_value,
            )
            for index in range(9)
        )

    profile_points = (
        horizontal(0.55),
        vertical(0.75, 0.55, -0.55),
        horizontal(-0.55, reverse=True),
        vertical(-0.75, -0.55, 0.55),
    )
    endpoint_ids = ((0, 1), (1, 2), (2, 3), (3, 0))
    deformer = cmds.deformer(
        mesh,
        type="curvenetNode",
        name="linuxCurvenetNode",
    )[0]
    if cmds.getAttr(deformer + ".nodeState") != 1:
        return _fail("new Curvenet node was not blocked during construction")

    arguments = [deformer, mesh, False]
    for curve_id, (points, node_ids) in enumerate(
        zip(profile_points, endpoint_ids)
    ):
        coordinates = ",".join(
            format(component, ".17g")
            for point in points
            for component in point
        )
        cmds.setAttr(
            "{}.inputCurveCoordinates[{}]".format(deformer, curve_id),
            coordinates,
            type="string",
        )
        cmds.setAttr(
            "{}.inputCurveStartNodeIds[{}]".format(deformer, curve_id),
            node_ids[0],
        )
        cmds.setAttr(
            "{}.inputCurveEndNodeIds[{}]".format(deformer, curve_id),
            node_ids[1],
        )
        arguments.extend((coordinates, node_ids[0], node_ids[1]))

    print("CURVENET_LINUX_STAGE: initialize", flush=True)
    cmds.initializeCurvenetEmbedding(*arguments)
    print("CURVENET_LINUX_STAGE: initialized", flush=True)

    # Exercise live DG evaluation as well as isolated embedding preparation.
    frame_points = []
    frame_ids = []
    corners = [points[0] for points in profile_points]
    for node_id, point in enumerate(corners):
        x, y, z = point
        frame_points.extend(
            ((x, y, z), (x + 0.01, y, z), (x, y + 0.01, z), (x, y, z + 0.01))
        )
        frame_ids.extend((node_id,) * 4)
    driver = cmds.curve(name="linuxCurvenetDriver", degree=1, point=frame_points)
    driver_shape = cmds.listRelatives(driver, shapes=True, fullPath=True)[0]
    for index, node_id in enumerate(frame_ids):
        cmds.setAttr(
            "{}.inputDriverNodeIds[{}]".format(deformer, index),
            node_id,
        )
    cmds.connectAttr(driver_shape + ".local", deformer + ".inputDriverCurve")
    cmds.setAttr(deformer + ".nodeState", 0)
    cmds.dgdirty(deformer)
    # Force evaluation through Maya's mesh API. Querying worldMesh with
    # getAttr prints a misleading error because mesh data is not scalar.
    import maya.api.OpenMaya as om

    selection = om.MSelectionList()
    selection.add(mesh)
    mesh_path = selection.getDagPath(0)
    mesh_path.extendToShape()
    evaluated_points = om.MFnMesh(mesh_path).getPoints(om.MSpace.kObject)
    if not evaluated_points:
        return _fail("live deformation produced no mesh points")
    print("CURVENET_LINUX_STAGE: evaluated", flush=True)

    # Exercise the same Python authoring/finish path used by the UI. This is
    # deliberately separate from the direct API test above so its stage label
    # identifies a workflow-layer failure unambiguously.
    cmds.file(new=True, force=True)
    workflow_mesh = cmds.polyCylinder(
        name="linuxWorkflowTube",
        radius=radius,
        height=5.0,
        subdivisionsAxis=24,
        subdivisionsHeight=18,
        subdivisionsCaps=1,
        createUVs=3,
    )[0]
    cmds.delete(workflow_mesh, constructionHistory=True)
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.environ["CURVENET_PROJECT_DIR"] = project_dir
    workflow_path = os.path.join(project_dir, "maya", "curvenet_workflow.py")
    workflow = {"__file__": workflow_path, "__name__": "curvenet_linux_test"}
    with open(workflow_path, "r") as workflow_file:
        exec(
            compile(workflow_file.read(), workflow_path, "exec"),
            workflow,
        )

    cmds.select(workflow_mesh, replace=True)
    workflow["_configure_source_mesh"](workflow_mesh, False)
    workflow["ensure_groups"]()
    corner_positions = (
        profile_points[0][0],
        profile_points[0][-1],
        profile_points[2][0],
        profile_points[2][-1],
    )
    authored_nodes = [
        workflow["find_or_create_node"](position, reuse_existing=False)
        for position in corner_positions
    ]
    for start_index, end_index in ((0, 1), (1, 2), (2, 3), (3, 0)):
        workflow["create_curve_between_nodes"](
            authored_nodes[start_index],
            authored_nodes[end_index],
        )
    cmds.select(workflow_mesh, replace=True)
    print("CURVENET_LINUX_STAGE: workflow_finish", flush=True)
    workflow["finish_curvenet"](full_surface=False)
    print("CURVENET_LINUX_STAGE: workflow_finished", flush=True)
    print("CURVENET_LINUX_PIPELINE: PASS", flush=True)
    maya.standalone.uninitialize()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(_fail("uncaught exception"))
