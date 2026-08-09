import maya.api.OpenMaya as om
import maya.cmds as cmds


PROJECTED_SAMPLES = 80


def _mesh_shape(mesh):
    shapes = cmds.listRelatives(
        mesh,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    ) or []

    if not shapes:
        raise RuntimeError(f"No mesh shape found under {mesh}.")

    return shapes[0]


def _short_name(node):
    return node.rsplit("|", 1)[-1].replace(":", "_")


def _point_on_curve(curve, parameter):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]
    info = cmds.createNode("pointOnCurveInfo")

    try:
        cmds.connectAttr(
            shape + ".worldSpace[0]",
            info + ".inputCurve",
            force=True,
        )
        minimum = cmds.getAttr(shape + ".minValue")
        maximum = cmds.getAttr(shape + ".maxValue")
        value = minimum + (maximum - minimum) * parameter
        cmds.setAttr(info + ".parameter", value)
        return list(cmds.getAttr(info + ".position")[0])
    finally:
        cmds.delete(info)


def _endpoint_controls(curve):
    expression = curve + "_endpointExpr"

    if not cmds.objExists(expression):
        raise RuntimeError(
            f"No endpoint expression found for source curve {curve}."
        )

    expression_text = cmds.expression(
        expression,
        query=True,
        string=True,
    )
    controls = []

    for line in expression_text.splitlines():
        if ".xValue" not in line or ".controlPoints[" not in line:
            continue

        control = line.split("=")[1].strip().split(".")[0]

        if control not in controls:
            controls.append(control)

    if len(controls) != 2:
        raise RuntimeError(
            f"Could not identify both endpoint controls for {curve}."
        )

    return controls[0], controls[1]


def _authored_segments(source_curve_group):
    if not cmds.objExists(source_curve_group):
        raise RuntimeError(
            f"Source Curvenet group does not exist: {source_curve_group}"
        )

    curves = cmds.listRelatives(
        source_curve_group,
        children=True,
        type="transform",
        fullPath=False,
    ) or []

    authored = [
        curve
        for curve in curves
        if cmds.attributeQuery(
            "curvenetSegment",
            node=curve,
            exists=True,
        )
    ]

    if not authored:
        raise RuntimeError(
            f"No authored Curvenet segments found in {source_curve_group}."
        )

    return authored


def _world_matrix(node):
    values = cmds.xform(
        node,
        query=True,
        worldSpace=True,
        matrix=True,
    )
    return om.MMatrix(values)


def _transfer_world_point(
    point,
    source_inverse_matrix,
    target_world_matrix,
):
    source_local = om.MPoint(point) * source_inverse_matrix
    target_world = source_local * target_world_matrix
    return [target_world.x, target_world.y, target_world.z]


def _project_world_point(point, target_mesh):
    target_shape = _mesh_shape(target_mesh)
    closest_point = cmds.createNode("closestPointOnMesh")

    try:
        cmds.connectAttr(
            target_shape + ".worldMesh[0]",
            closest_point + ".inMesh",
            force=True,
        )
        cmds.connectAttr(
            target_shape + ".worldMatrix[0]",
            closest_point + ".inputMatrix",
            force=True,
        )
        cmds.setAttr(
            closest_point + ".inPosition",
            point[0],
            point[1],
            point[2],
            type="double3",
        )
        return list(cmds.getAttr(closest_point + ".position")[0])
    finally:
        cmds.delete(closest_point)


def _deformer_input_mesh_plug(deformer):
    sources = cmds.listConnections(
        deformer + ".input[0].inputGeometry",
        source=True,
        destination=False,
        plugs=True,
    ) or []

    if not sources:
        raise RuntimeError(
            f"Could not find the upstream mesh connected to {deformer}."
        )

    source_node = sources[0].split(".", 1)[0]
    return source_node + ".worldMesh[0]"


def _create_target_groups(target_prefix):
    root_group = target_prefix + "_transferredCurvenet_GRP"

    if cmds.objExists(root_group):
        cmds.delete(root_group)

    root_group = cmds.group(empty=True, name=root_group)
    node_group = cmds.group(
        empty=True,
        name=target_prefix + "_transferredCurvenet_nodes_GRP",
        parent=root_group,
    )
    projected_group = cmds.group(
        empty=True,
        name=target_prefix + "_transferredCurvenet_projectedCurves_GRP",
        parent=root_group,
    )
    return root_group, node_group, projected_group


def _create_node_marker(target_prefix, node_id, position, node_group):
    marker = cmds.polySphere(
        name=f"{target_prefix}_CN_node_{node_id}",
        radius=0.07,
        subdivisionsX=12,
        subdivisionsY=12,
    )[0]
    cmds.xform(marker, worldSpace=True, translation=position)
    cmds.parent(marker, node_group)
    cmds.addAttr(
        marker,
        longName="transferredCurvenetNode",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(marker + ".transferredCurvenetNode", lock=True)
    return marker


def _create_endpoint_expression(curve, start_marker, end_marker):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=False,
    )[0]
    last_control = cmds.getAttr(shape + ".controlPoints", size=True) - 1

    lines = []

    for control_index, marker in (
        (0, start_marker),
        (last_control, end_marker),
    ):
        lines.extend([
            f"{shape}.controlPoints[{control_index}].xValue = "
            f"{marker}.translateX;",
            f"{shape}.controlPoints[{control_index}].yValue = "
            f"{marker}.translateY;",
            f"{shape}.controlPoints[{control_index}].zValue = "
            f"{marker}.translateZ;",
        ])

    cmds.expression(
        name=curve + "_endpointExpr",
        string="\n".join(lines),
        alwaysEvaluate=True,
        unitConversion="all",
    )


def attach_existing_curvenet_to_mesh(
    target_mesh,
    source_mesh="tubeA",
    source_curve_group="tubeA_drawnCurvenet_curves_GRP",
):
    """Transfer one authored Curvenet to another mesh and bind it."""
    if not cmds.objExists(source_mesh):
        raise RuntimeError(f"Source mesh does not exist: {source_mesh}")

    if not cmds.objExists(target_mesh):
        raise RuntimeError(f"Target mesh does not exist: {target_mesh}")

    source_segments = _authored_segments(source_curve_group)
    target_prefix = _short_name(target_mesh)
    _, node_group, projected_group = _create_target_groups(target_prefix)

    source_inverse_matrix = _world_matrix(source_mesh).inverse()
    target_world_matrix = _world_matrix(target_mesh)
    projected_endpoint_by_control = {}
    marker_by_control = {}
    projected_curves = []

    def projected_endpoint(control):
        if control not in projected_endpoint_by_control:
            source_position = cmds.xform(
                control,
                query=True,
                worldSpace=True,
                translation=True,
            )
            transferred = _transfer_world_point(
                source_position,
                source_inverse_matrix,
                target_world_matrix,
            )
            projected_endpoint_by_control[control] = _project_world_point(
                transferred,
                target_mesh,
            )
            marker_by_control[control] = _create_node_marker(
                target_prefix,
                len(projected_endpoint_by_control) - 1,
                projected_endpoint_by_control[control],
                node_group,
            )

        return projected_endpoint_by_control[control]

    for curve_id, source_curve in enumerate(source_segments):
        start_control, end_control = _endpoint_controls(source_curve)
        points = [projected_endpoint(start_control)]

        for sample_index in range(1, PROJECTED_SAMPLES - 1):
            parameter = sample_index / float(PROJECTED_SAMPLES - 1)
            source_point = _point_on_curve(source_curve, parameter)
            transferred = _transfer_world_point(
                source_point,
                source_inverse_matrix,
                target_world_matrix,
            )
            points.append(
                _project_world_point(transferred, target_mesh)
            )

        points.append(projected_endpoint(end_control))

        projected_curve = cmds.curve(
            name=f"{target_prefix}_CN_projected_{curve_id}",
            degree=1,
            point=points,
        )
        cmds.parent(projected_curve, projected_group)
        cmds.addAttr(
            projected_curve,
            longName="projectedCurvenetProfile",
            attributeType="bool",
            defaultValue=True,
        )
        cmds.setAttr(
            projected_curve + ".projectedCurvenetProfile",
            lock=True,
        )
        _create_endpoint_expression(
            projected_curve,
            marker_by_control[start_control],
            marker_by_control[end_control],
        )
        projected_curves.append(projected_curve)

    deformer_name = target_prefix + "CurvenetNode"
    preview_group = deformer_name + "_curvenet_group"

    if cmds.objExists(deformer_name):
        cmds.delete(deformer_name)

    if cmds.objExists(preview_group):
        cmds.delete(preview_group)

    deformer = cmds.deformer(
        target_mesh,
        type="curvenetNode",
        name=deformer_name,
    )[0]
    cmds.connectAttr(
        _deformer_input_mesh_plug(deformer),
        deformer + ".inputMesh",
        force=True,
    )

    for curve_id, curve in enumerate(projected_curves):
        curve_shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]
        cmds.connectAttr(
            curve_shape + ".worldSpace[0]",
            f"{deformer}.inputCurves[{curve_id}]",
            force=True,
        )

    print("Transferred Curvenet to:", target_mesh)
    print("Projected curves:", len(projected_curves))
    print("Shared projected endpoint controls:", len(projected_endpoint_by_control))
    print("Deformer:", deformer)
    return deformer


def attach_existing_curvenet_to_selected_mesh():
    """Attach Tube A's authored Curvenet to the selected target mesh."""
    selection = cmds.ls(selection=True, long=True, type="transform") or []

    if len(selection) != 1:
        raise RuntimeError("Select exactly one target mesh transform.")

    target_mesh = selection[0]
    _mesh_shape(target_mesh)
    return attach_existing_curvenet_to_mesh(target_mesh)
