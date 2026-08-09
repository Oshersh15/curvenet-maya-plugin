import re

import maya.cmds as cmds


CONSTRAINT_ATTRIBUTE = "curvenetJointConstraint"
BIND_MATRIX_ATTRIBUTE = "curvenetBindWorldMatrix"
CURVE_SKIN_ATTRIBUTE = "curvenetJointSkinCluster"
NODE_MARKER_ATTRIBUTES = (
    "curvenetNode",
    "transferredCurvenetNode",
)


def _is_curvenet_node(node):
    return any(
        cmds.attributeQuery(attribute, node=node, exists=True)
        for attribute in NODE_MARKER_ATTRIBUTES
    )


def _selected_curvenet_nodes(selection):
    nodes = []

    for selected in selection:
        candidates = [selected]
        candidates.extend(
            cmds.listRelatives(
                selected,
                allDescendents=True,
                type="transform",
                fullPath=True,
            ) or []
        )

        for candidate in candidates:
            if _is_curvenet_node(candidate) and candidate not in nodes:
                nodes.append(candidate)

    return nodes


def _joint_depth(joint):
    return len(cmds.ls(joint, long=True)[0].split("|"))


def _world_position(node):
    return cmds.xform(
        node,
        query=True,
        worldSpace=True,
        translation=True,
    )


def _point_segment_weights(point, start, end):
    direction = [end[index] - start[index] for index in range(3)]
    length_squared = sum(component * component for component in direction)

    if length_squared <= 1.0e-12:
        parameter = 0.0
    else:
        relative = [point[index] - start[index] for index in range(3)]
        parameter = sum(
            relative[index] * direction[index]
            for index in range(3)
        ) / length_squared
        parameter = max(0.0, min(1.0, parameter))

    closest = [
        start[index] + parameter * direction[index]
        for index in range(3)
    ]
    distance_squared = sum(
        (point[index] - closest[index]) ** 2
        for index in range(3)
    )
    return distance_squared, 1.0 - parameter, parameter


def _nearest_bone_weights_at_position(point, joint_positions):
    if len(joint_positions) == 1:
        return [1.0]

    best = None

    for segment_index in range(len(joint_positions) - 1):
        result = _point_segment_weights(
            point,
            joint_positions[segment_index],
            joint_positions[segment_index + 1],
        )

        if best is None or result[0] < best[0]:
            best = (result[0], segment_index, result[1], result[2])

    weights = [0.0] * len(joint_positions)
    weights[best[1]] = best[2]
    weights[best[1] + 1] = best[3]
    return weights


def _nearest_bone_weights(node, joints):
    return _nearest_bone_weights_at_position(
        _world_position(node),
        [_world_position(joint) for joint in joints],
    )


def _curve_endpoint_controls(curve):
    expression = curve.rsplit("|", 1)[-1] + "_endpointExpr"

    if not cmds.objExists(expression):
        return []

    text = cmds.expression(expression, query=True, string=True)
    controls = []

    for control in re.findall(r"([|:\w]+)\.translate[XYZ]", text):
        short_name = control.rsplit("|", 1)[-1]

        if short_name not in controls:
            controls.append(short_name)

    return controls


def _projected_curves_for_nodes(nodes):
    node_names = {node.rsplit("|", 1)[-1] for node in nodes}
    curves = []

    for attribute in cmds.ls("*.projectedCurvenetProfile", long=True) or []:
        curve = attribute.rsplit(".", 1)[0]
        controls = _curve_endpoint_controls(curve)

        if len(controls) == 2 and set(controls).issubset(node_names):
            curves.append(curve)

    return curves


def _connected_curve_skin(curve):
    if not cmds.attributeQuery(CURVE_SKIN_ATTRIBUTE, node=curve, exists=True):
        return None

    connections = cmds.listConnections(
        curve + "." + CURVE_SKIN_ATTRIBUTE,
        source=True,
        destination=False,
        type="skinCluster",
    ) or []
    return connections[0] if connections else None


def _skin_projected_curves(curves, joints):
    joint_positions = [_world_position(joint) for joint in joints]

    for curve in curves:
        existing_skin = _connected_curve_skin(curve)

        if existing_skin:
            cmds.delete(existing_skin)

        expression = curve.rsplit("|", 1)[-1] + "_endpointExpr"

        if cmds.objExists(expression):
            cmds.delete(expression)

        skin = cmds.skinCluster(
            joints,
            curve,
            toSelectedBones=True,
            bindMethod=0,
            skinMethod=0,
            normalizeWeights=1,
            maximumInfluences=2,
            obeyMaxInfluences=True,
            name=curve.rsplit("|", 1)[-1] + "_curvenetJointSkinCluster",
        )[0]
        shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]
        control_count = cmds.getAttr(shape + ".controlPoints", size=True)

        for control_index in range(control_count):
            component = f"{curve}.cv[{control_index}]"
            position = cmds.pointPosition(component, world=True)
            weights = _nearest_bone_weights_at_position(
                position,
                joint_positions,
            )
            cmds.skinPercent(
                skin,
                component,
                transformValue=list(zip(joints, weights)),
                normalize=True,
            )

        if not cmds.attributeQuery(CURVE_SKIN_ATTRIBUTE, node=curve, exists=True):
            cmds.addAttr(
                curve,
                longName=CURVE_SKIN_ATTRIBUTE,
                attributeType="message",
            )

        cmds.connectAttr(
            skin + ".message",
            curve + "." + CURVE_SKIN_ATTRIBUTE,
            force=True,
        )

    return curves


def _unskin_projected_curves(nodes):
    prefixes = {
        node.rsplit("|", 1)[-1].split("_CN_node_", 1)[0]
        for node in nodes
        if "_CN_node_" in node.rsplit("|", 1)[-1]
    }
    removed = 0

    for attribute in cmds.ls("*." + CURVE_SKIN_ATTRIBUTE, long=True) or []:
        curve = attribute.rsplit(".", 1)[0]
        short_name = curve.rsplit("|", 1)[-1]

        if not any(short_name.startswith(prefix + "_") for prefix in prefixes):
            continue

        skin = _connected_curve_skin(curve)

        if skin:
            cmds.delete(skin)
            removed += 1

    return removed


def _connected_constraint(node):
    if not cmds.attributeQuery(CONSTRAINT_ATTRIBUTE, node=node, exists=True):
        return None

    connections = cmds.listConnections(
        node + "." + CONSTRAINT_ATTRIBUTE,
        source=True,
        destination=False,
        type="parentConstraint",
    ) or []
    return connections[0] if connections else None


def _store_bind_matrix(node):
    if not cmds.attributeQuery(BIND_MATRIX_ATTRIBUTE, node=node, exists=True):
        cmds.addAttr(node, longName=BIND_MATRIX_ATTRIBUTE, dataType="matrix")

    matrix = cmds.xform(
        node,
        query=True,
        worldSpace=True,
        matrix=True,
    )
    cmds.setAttr(
        node + "." + BIND_MATRIX_ATTRIBUTE,
        *matrix,
        type="matrix",
    )


def _restore_bind_matrix(node):
    if not cmds.attributeQuery(BIND_MATRIX_ATTRIBUTE, node=node, exists=True):
        return

    matrix = cmds.getAttr(node + "." + BIND_MATRIX_ATTRIBUTE)
    cmds.xform(node, worldSpace=True, matrix=matrix)


def bind_selected_curvenet_nodes_to_joints():
    selection = cmds.ls(selection=True, long=True) or []
    joints = sorted(
        [node for node in selection if cmds.nodeType(node) == "joint"],
        key=_joint_depth,
    )
    nodes = _selected_curvenet_nodes(selection)

    if not joints:
        raise RuntimeError("Select at least one joint.")

    if not nodes:
        raise RuntimeError("Select Curvenet spheres or their nodes group.")

    for node in nodes:
        existing_constraint = _connected_constraint(node)

        if existing_constraint:
            cmds.delete(existing_constraint)

        _store_bind_matrix(node)
        constraint = cmds.parentConstraint(
            joints,
            node,
            maintainOffset=True,
            name=node.rsplit("|", 1)[-1] + "_curvenetJointConstraint",
        )[0]
        weights = _nearest_bone_weights(node, joints)
        aliases = cmds.parentConstraint(
            constraint,
            query=True,
            weightAliasList=True,
        )

        for alias, weight in zip(aliases, weights):
            cmds.setAttr(constraint + "." + alias, weight)

        if not cmds.attributeQuery(CONSTRAINT_ATTRIBUTE, node=node, exists=True):
            cmds.addAttr(
                node,
                longName=CONSTRAINT_ATTRIBUTE,
                attributeType="message",
            )

        cmds.connectAttr(
            constraint + ".message",
            node + "." + CONSTRAINT_ATTRIBUTE,
            force=True,
        )

    skinned_curves = _skin_projected_curves(
        _projected_curves_for_nodes(nodes),
        joints,
    )
    print("Bound Curvenet nodes:", len(nodes))
    print("Skinned projected curves:", len(skinned_curves))
    print("Joint influences:", len(joints))
    return nodes


def unbind_selected_curvenet_nodes():
    selection = cmds.ls(selection=True, long=True) or []
    nodes = _selected_curvenet_nodes(selection)

    if not nodes:
        raise RuntimeError("Select Curvenet spheres or their nodes group.")

    unbound = 0

    for node in nodes:
        constraint = _connected_constraint(node)

        if not constraint:
            continue

        cmds.delete(constraint)
        _restore_bind_matrix(node)
        unbound += 1

    unskinned = _unskin_projected_curves(nodes)
    print("Unbound Curvenet nodes:", unbound)
    print("Unskinned projected curves:", unskinned)
    return nodes


print("Curvenet joint binding enabled.")
