"""Interactive authoring of a Curvenet on Tube A.

Load this through curvenet_workflow.py rather than executing it directly.
"""

import math
import maya.cmds as cmds
import maya.api.OpenMaya as om
import maya.api.OpenMayaUI as omui


MESH_NAME = "tubeA"
DEFORMER_NAME = "tubeACurvenetNode"

ROOT_GRP = "tubeA_drawnCurvenet_GRP"
NODE_GRP = "tubeA_drawnCurvenet_nodes_GRP"
CURVE_GRP = "tubeA_drawnCurvenet_curves_GRP"
PROJECTED_GRP = "tubeA_drawnCurvenet_projectedCurves_GRP"

NODE_PREFIX = "tubeA_CN_node_"
CURVE_PREFIX = "tubeA_CN_segment_"
PROJECTED_PREFIX = "tubeA_CN_projected_"

DRAW_CONTEXT = "tubeA_curvenetDrawContext"

SNAP_DISTANCE = 0.18
PROJECTED_SAMPLES = 80

_pending_node = None


def ensure_groups():
    if not cmds.objExists(ROOT_GRP):
        cmds.group(empty=True, name=ROOT_GRP)

    for group in [NODE_GRP, CURVE_GRP, PROJECTED_GRP]:
        if not cmds.objExists(group):
            cmds.group(empty=True, name=group)
            cmds.parent(group, ROOT_GRP)


def next_index(prefix):
    return len(cmds.ls(prefix + "*") or [])


def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


def mesh_shape():
    shapes = cmds.listRelatives(
        MESH_NAME,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )

    if not shapes:
        raise RuntimeError(f"No mesh shape found under {MESH_NAME}")

    return shapes[0]


def get_world_position(obj):
    return cmds.xform(
        obj,
        query=True,
        worldSpace=True,
        translation=True,
    )


def existing_nodes():
    return cmds.ls(NODE_PREFIX + "*", type="transform") or []


def project_world_point_to_mesh(point):
    shape = mesh_shape()

    node = cmds.createNode("closestPointOnMesh")

    cmds.connectAttr(shape + ".worldMesh[0]", node + ".inMesh", force=True)
    cmds.connectAttr(shape + ".worldMatrix[0]", node + ".inputMatrix", force=True)

    cmds.setAttr(
        node + ".inPosition",
        point[0],
        point[1],
        point[2],
        type="double3",
    )

    projected = cmds.getAttr(node + ".position")[0]

    cmds.delete(node)

    return list(projected)


def raycast_mesh_from_view(x, y):
    selection = om.MSelectionList()
    selection.add(MESH_NAME)

    dag_path = selection.getDagPath(0)
    dag_path.extendToShape()

    mesh_fn = om.MFnMesh(dag_path)

    view = omui.M3dView.active3dView()

    near_point = om.MPoint()
    far_point = om.MPoint()

    view.viewToWorld(
        int(x),
        int(y),
        near_point,
        far_point,
    )

    direction = far_point - near_point
    direction.normalize()

    hit = mesh_fn.closestIntersection(
        om.MFloatPoint(near_point),
        om.MFloatVector(direction),
        om.MSpace.kWorld,
        999999.0,
        False,
    )

    if hit is None:
        return None

    hit_point = hit[0]

    return [hit_point.x, hit_point.y, hit_point.z]


def find_or_create_node(position):
    ensure_groups()

    for node in existing_nodes():
        node_pos = get_world_position(node)

        if distance(position, node_pos) <= SNAP_DISTANCE:
            return node

    node_id = next_index(NODE_PREFIX)

    node = cmds.polySphere(
        name=f"{NODE_PREFIX}{node_id}",
        radius=0.07,
        subdivisionsX=12,
        subdivisionsY=12,
    )[0]

    cmds.xform(node, worldSpace=True, translation=position)
    cmds.parent(node, NODE_GRP)

    cmds.addAttr(node, longName="curvenetNode", attributeType="bool", defaultValue=True)
    cmds.setAttr(node + ".curvenetNode", lock=True)

    return node


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


def create_endpoint_expression(curve, start_node, end_node):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=False,
    )[0]

    expr_name = curve + "_endpointExpr"

    if cmds.objExists(expr_name):
        cmds.delete(expr_name)

    lines = [
        f"{shape}.controlPoints[0].xValue = {start_node}.translateX;",
        f"{shape}.controlPoints[0].yValue = {start_node}.translateY;",
        f"{shape}.controlPoints[0].zValue = {start_node}.translateZ;",
        f"{shape}.controlPoints[3].xValue = {end_node}.translateX;",
        f"{shape}.controlPoints[3].yValue = {end_node}.translateY;",
        f"{shape}.controlPoints[3].zValue = {end_node}.translateZ;",
    ]

    cmds.expression(
        name=expr_name,
        string="\n".join(lines),
        alwaysEvaluate=True,
        unitConversion="all",
    )


def create_curve_between_nodes(start_node, end_node):
    ensure_groups()

    p0 = get_world_position(start_node)
    p3 = get_world_position(end_node)

    p1 = project_world_point_to_mesh(lerp(p0, p3, 1.0 / 3.0))
    p2 = project_world_point_to_mesh(lerp(p0, p3, 2.0 / 3.0))

    curve_id = next_index(CURVE_PREFIX)

    curve = cmds.curve(
        name=f"{CURVE_PREFIX}{curve_id}",
        degree=3,
        point=[p0, p1, p2, p3],
    )

    cmds.parent(curve, CURVE_GRP)

    cmds.addAttr(curve, longName="curvenetSegment", attributeType="bool", defaultValue=True)
    cmds.setAttr(curve + ".curvenetSegment", lock=True)

    create_endpoint_expression(curve, start_node, end_node)

    print("Created Curvenet segment:", curve)

    return curve


def on_curvenet_click():
    global _pending_node

    pos = cmds.draggerContext(
        DRAW_CONTEXT,
        query=True,
        anchorPoint=True,
    )

    hit = raycast_mesh_from_view(pos[0], pos[1])

    if hit is None:
        print("No hit on tubeA.")
        return

    node = find_or_create_node(hit)

    if _pending_node is None:
        _pending_node = node
        print("Start node:", node)
        return

    if node == _pending_node:
        print("Clicked the same Curvenet node. Choose another point.")
        return

    create_curve_between_nodes(_pending_node, node)
    _pending_node = None


def start_curvenet_draw_tool():
    global _pending_node

    ensure_groups()
    _pending_node = None

    if cmds.draggerContext(DRAW_CONTEXT, exists=True):
        cmds.deleteUI(DRAW_CONTEXT)

    cmds.draggerContext(
        DRAW_CONTEXT,
        pressCommand=on_curvenet_click,
        cursor="crossHair",
        space="screen",
    )

    cmds.setToolTo(DRAW_CONTEXT)

    print("Curvenet draw tool active.")
    print("Click two points on tubeA to create one segment.")


def stop_curvenet_draw_tool():
    global _pending_node

    _pending_node = None
    cmds.setToolTo("selectSuperContext")
    print("Curvenet draw tool stopped.")


def authored_segments():
    ensure_groups()

    curves = cmds.listRelatives(
        CURVE_GRP,
        children=True,
        type="transform",
        fullPath=False,
    ) or []

    return [
        curve
        for curve in curves
        if cmds.attributeQuery("curvenetSegment", node=curve, exists=True)
    ]


def point_on_curve(curve, t):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]

    info = cmds.createNode("pointOnCurveInfo")

    cmds.connectAttr(shape + ".worldSpace[0]", info + ".inputCurve", force=True)

    min_param = cmds.getAttr(shape + ".minValue")
    max_param = cmds.getAttr(shape + ".maxValue")

    param = min_param + (max_param - min_param) * t

    cmds.setAttr(info + ".parameter", param)

    pos = cmds.getAttr(info + ".position")[0]

    cmds.delete(info)

    return list(pos)


def get_curve_endpoint_controls(curve):
    expr = curve + "_endpointExpr"

    if not cmds.objExists(expr):
        raise RuntimeError(f"No endpoint expression found for {curve}.")

    expr_text = cmds.expression(expr, query=True, string=True)

    controls = []

    for line in expr_text.splitlines():
        if ".controlPoints[0].xValue" in line:
            controls.append(line.split("=")[1].strip().split(".")[0])

        if ".controlPoints[3].xValue" in line:
            controls.append(line.split("=")[1].strip().split(".")[0])

    if len(controls) != 2:
        raise RuntimeError(f"Could not read endpoint controls for {curve}.")

    return controls[0], controls[1]


def cached_projected_control_position(control, cache):
    if control in cache:
        return cache[control]

    pos = get_world_position(control)
    projected = project_world_point_to_mesh(pos)

    cache[control] = projected

    return projected


def delete_existing_projected_curves():
    ensure_groups()

    children = cmds.listRelatives(
        PROJECTED_GRP,
        children=True,
        type="transform",
        fullPath=False,
    ) or []

    if children:
        cmds.delete(children)


def create_projected_curve(curve, index, cache):
    start_control, end_control = get_curve_endpoint_controls(curve)

    points = []

    points.append(cached_projected_control_position(start_control, cache))

    for sample_index in range(1, PROJECTED_SAMPLES - 1):
        t = sample_index / float(PROJECTED_SAMPLES - 1)

        raw = point_on_curve(curve, t)
        projected = project_world_point_to_mesh(raw)

        points.append(projected)

    points.append(cached_projected_control_position(end_control, cache))

    projected_curve = cmds.curve(
        name=f"{PROJECTED_PREFIX}{index}",
        degree=1,
        point=points,
    )

    cmds.parent(projected_curve, PROJECTED_GRP)

    cmds.addAttr(
        projected_curve,
        longName="projectedCurvenetProfile",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(projected_curve + ".projectedCurvenetProfile", lock=True)

    return projected_curve


def build_projected_curves():
    ensure_groups()
    delete_existing_projected_curves()

    curves = authored_segments()

    if not curves:
        raise RuntimeError("No authored Curvenet segments found.")

    cache = {}
    projected = []

    for index, curve in enumerate(curves):
        projected_curve = create_projected_curve(curve, index, cache)
        projected.append(projected_curve)
        print("Projected", curve, "->", projected_curve)

    print("Shared projected endpoint controls:", len(cache))

    return projected


def connect_drawn_curvenet_to_plugin():
    ensure_groups()

    if cmds.objExists(DEFORMER_NAME):
        cmds.delete(DEFORMER_NAME)

    preview_group = DEFORMER_NAME + "_curvenet_group"

    if cmds.objExists(preview_group):
        cmds.delete(preview_group)

    projected_curves = build_projected_curves()

    deformer = cmds.deformer(
        MESH_NAME,
        type="curvenetNode",
        name=DEFORMER_NAME,
    )[0]

    cmds.connectAttr(
        mesh_shape() + ".outMesh",
        deformer + ".inputMesh",
        force=True,
    )

    for curve_id, curve in enumerate(projected_curves):
        shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]

        cmds.connectAttr(
            shape + ".worldSpace[0]",
            f"{deformer}.inputCurves[{curve_id}]",
            force=True,
        )

        print(f"Logical profile ID {curve_id}:", curve)

    print("\nConnected projected Curvenet to plugin.")
    print("Projected curves:", len(projected_curves))
    print("Deformer:", deformer)

    return deformer
