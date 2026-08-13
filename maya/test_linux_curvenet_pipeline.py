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
    cmds.setAttr(deformer + ".nodeState", 1)

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
    cmds.getAttr(mesh + ".worldMesh[0]")
    print("CURVENET_LINUX_STAGE: evaluated", flush=True)
    print("CURVENET_LINUX_PIPELINE: PASS", flush=True)
    maya.standalone.uninitialize()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(_fail("uncaught exception"))
