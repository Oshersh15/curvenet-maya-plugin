import re
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

        for control in re.findall(r"([|:\w]+)\.translate[XYZ]", line):
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


def _joint_hierarchy(root):
    joints = []

    def visit(joint):
        joints.append(joint)

        for child in cmds.listRelatives(
            joint,
            children=True,
            type="joint",
            fullPath=True,
        ) or []:
            visit(child)

    visit(cmds.ls(root, long=True)[0])
    return joints


def transfer_joint_hierarchy_to_mesh(
    source_root_joint,
    target_mesh,
    source_mesh="tubeA",
    connect_pose=True,
):
    """Duplicate a source skeleton into a target mesh's local frame."""
    if not cmds.objExists(source_root_joint):
        raise RuntimeError(f"Source root joint does not exist: {source_root_joint}")

    if not cmds.objExists(source_mesh):
        raise RuntimeError(f"Source mesh does not exist: {source_mesh}")

    if not cmds.objExists(target_mesh):
        raise RuntimeError(f"Target mesh does not exist: {target_mesh}")

    target_prefix = _short_name(target_mesh)
    target_root_name = target_prefix + "_skeleton_root"
    target_group_name = target_prefix + "_transferredSkeleton_GRP"

    if cmds.objExists(target_root_name) or cmds.objExists(target_group_name):
        raise RuntimeError(
            f"Target skeleton already exists for: {target_prefix}"
        )

    source_joints = _joint_hierarchy(source_root_joint)
    duplicated_root = cmds.duplicate(
        source_root_joint,
        renameChildren=True,
        returnRootsOnly=True,
    )[0]
    duplicated_root = cmds.rename(duplicated_root, target_root_name)
    target_group = cmds.group(empty=True, name=target_group_name)
    duplicated_root = cmds.parent(
        duplicated_root,
        target_group,
        absolute=True,
    )[0]
    target_joints = _joint_hierarchy(duplicated_root)

    if len(source_joints) != len(target_joints):
        cmds.delete(duplicated_root)
        raise RuntimeError("Duplicated joint hierarchy does not match the source.")

    source_inverse_matrix = _world_matrix(source_mesh).inverse()
    target_world_matrix = _world_matrix(target_mesh)
    transfer_matrix = source_inverse_matrix * target_world_matrix
    cmds.xform(
        target_group,
        worldSpace=True,
        matrix=list(transfer_matrix),
    )
    duplicated_root = cmds.ls(duplicated_root, long=True)[0]
    target_joints = _joint_hierarchy(duplicated_root)

    if connect_pose:
        for source_joint, target_joint in zip(source_joints, target_joints):
            cmds.connectAttr(
                source_joint + ".rotate",
                target_joint + ".rotate",
                force=True,
            )
            cmds.connectAttr(
                source_joint + ".scale",
                target_joint + ".scale",
                force=True,
            )

    cmds.addAttr(
        target_group,
        longName="transferredCurvenetSkeleton",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(
        target_group + ".transferredCurvenetSkeleton",
        lock=True,
    )
    print("Transferred skeleton to:", target_mesh)
    print("Transferred joints:", len(target_joints))
    print("Target root:", duplicated_root)
    return duplicated_root, target_joints


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


def _create_node_marker(
    target_prefix,
    node_id,
    position,
    node_group,
    radius,
):
    marker = cmds.polySphere(
        name=f"{target_prefix}_CN_node_{node_id}",
        radius=radius,
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
    cmds.addAttr(
        marker,
        longName="curvenetLogicalNodeId",
        attributeType="long",
    )
    cmds.setAttr(marker + ".curvenetLogicalNodeId", node_id)
    cmds.setAttr(marker + ".curvenetLogicalNodeId", lock=True)
    return marker


def _source_logical_node_id(control):
    attribute = control + ".curvenetLogicalNodeId"

    if cmds.objExists(attribute):
        return cmds.getAttr(attribute)

    match = re.search(r"(\d+)$", control.rsplit("|", 1)[-1])

    if not match:
        raise RuntimeError(f"No logical Curvenet node ID found for {control}.")

    return int(match.group(1))


def _create_endpoint_expression(curve, start_marker, end_marker):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=False,
    )[0]
    control_count = cmds.getAttr(shape + ".controlPoints", size=True)
    last_control = control_count - 1
    start_rest = cmds.getAttr(start_marker + ".translate")[0]
    end_rest = cmds.getAttr(end_marker + ".translate")[0]

    lines = []

    for control_index in range(control_count):
        parameter = control_index / float(last_control) if last_control else 0.0
        start_weight = 1.0 - parameter
        end_weight = parameter
        rest = cmds.getAttr(
            f"{shape}.controlPoints[{control_index}]"
        )[0]

        for axis, component in zip("XYZ", range(3)):
            lines.append(
                f"{shape}.controlPoints[{control_index}].{axis.lower()}Value = "
                f"{rest[component]:.17g} + "
                f"{start_weight:.17g} * "
                f"({start_marker}.translate{axis} - "
                f"{start_rest[component]:.17g}) + "
                f"{end_weight:.17g} * "
                f"({end_marker}.translate{axis} - "
                f"{end_rest[component]:.17g});"
            )

    cmds.expression(
        name=curve + "_endpointExpr",
        string="\n".join(lines),
        alwaysEvaluate=False,
        unitConversion="all",
    )


def attach_existing_curvenet_to_mesh(
    target_mesh,
    source_mesh="tubeA",
    source_curve_group="tubeA_drawnCurvenet_curves_GRP",
    full_surface=False,
):
    """Transfer one authored Curvenet to another mesh and bind it."""
    if not cmds.objExists(source_mesh):
        raise RuntimeError(f"Source mesh does not exist: {source_mesh}")

    if not cmds.objExists(target_mesh):
        raise RuntimeError(f"Target mesh does not exist: {target_mesh}")

    source_segments = _authored_segments(source_curve_group)
    target_prefix = _short_name(target_mesh)
    _, node_group, projected_group = _create_target_groups(target_prefix)

    source_controls = []

    for source_curve in source_segments:
        for control in _endpoint_controls(source_curve):
            if control not in source_controls:
                source_controls.append(control)

    source_bounds = cmds.exactWorldBoundingBox(source_mesh)
    target_bounds = cmds.exactWorldBoundingBox(target_mesh)
    source_diagonal = sum(
        (source_bounds[index + 3] - source_bounds[index]) ** 2
        for index in range(3)
    ) ** 0.5
    target_diagonal = sum(
        (target_bounds[index + 3] - target_bounds[index]) ** 2
        for index in range(3)
    ) ** 0.5
    source_radii = []

    for control in source_controls:
        bounds = cmds.exactWorldBoundingBox(control)
        source_radii.append(
            0.5 * (
                (bounds[3] - bounds[0])
                + (bounds[4] - bounds[1])
                + (bounds[5] - bounds[2])
            ) / 3.0
        )

    source_radius = (
        sum(source_radii) / len(source_radii)
        if source_radii else max(source_diagonal * 0.006, 1.0e-5)
    )
    marker_radius = source_radius * (
        target_diagonal / source_diagonal
        if source_diagonal > 1.0e-12 else 1.0
    )

    source_inverse_matrix = _world_matrix(source_mesh).inverse()
    target_world_matrix = _world_matrix(target_mesh)
    projected_endpoint_by_control = {}
    marker_by_control = {}
    node_id_by_control = {}
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
            logical_node_id = _source_logical_node_id(control)
            marker_by_control[control] = _create_node_marker(
                target_prefix,
                logical_node_id,
                projected_endpoint_by_control[control],
                node_group,
                marker_radius,
            )
            node_id_by_control[control] = logical_node_id

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
        for attribute, marker in (
            ("curvenetStartControl", marker_by_control[start_control]),
            ("curvenetEndControl", marker_by_control[end_control]),
        ):
            cmds.addAttr(
                projected_curve,
                longName=attribute,
                attributeType="message",
            )
            cmds.connectAttr(
                marker + ".message",
                projected_curve + "." + attribute,
                force=True,
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
    cmds.setAttr(
        deformer + ".fullSurfaceCurvenet",
        bool(full_surface),
    )
    for curve_id, curve in enumerate(projected_curves):
        curve_shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]
        flat_points = cmds.xform(
            curve + ".cv[*]",
            query=True,
            worldSpace=True,
            translation=True,
        )
        points = [
            tuple(flat_points[index:index + 3]) + (1.0,)
            for index in range(0, len(flat_points), 3)
        ]
        cmds.setAttr(
            f"{deformer}.inputCurvePoints[{curve_id}]",
            len(points),
            *points,
            type="pointArray",
        )
        source_curve = source_segments[curve_id]
        start_control, end_control = _endpoint_controls(source_curve)
        cmds.setAttr(
            f"{deformer}.inputCurveStartNodeIds[{curve_id}]",
            node_id_by_control[start_control],
        )
        cmds.setAttr(
            f"{deformer}.inputCurveEndNodeIds[{curve_id}]",
            node_id_by_control[end_control],
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
