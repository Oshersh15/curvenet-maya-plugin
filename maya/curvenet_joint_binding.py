import re

import maya.cmds as cmds
import maya.mel as mel


CONSTRAINT_ATTRIBUTE = "curvenetJointConstraint"
BIND_MATRIX_ATTRIBUTE = "curvenetBindWorldMatrix"
CURVE_SKIN_ATTRIBUTE = "curvenetJointSkinCluster"
JOINT_WEIGHTS_ATTRIBUTE = "curvenetJointWeights"
NODE_MARKER_ATTRIBUTES = (
    "curvenetNode",
    "transferredCurvenetNode",
)
DRIVER_MARKER_ATTRIBUTE = "curvenetWeightDriver"
DRIVER_FRAME_POINT_COUNT_ATTRIBUTE = "curvenetFramePointCount"
WEIGHT_CONTROL_ATTRIBUTE = "curvenetWeightControlIndex"
WEIGHT_EDITOR_WINDOW = "curvenetWeightEditorWindow"


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


def _nearest_bone_weights_at_position(point, joints):
    if len(joints) == 1:
        return [1.0]

    joint_positions = [_world_position(joint) for joint in joints]
    joint_index_by_name = {
        cmds.ls(joint, long=True)[0]: index
        for index, joint in enumerate(joints)
    }
    bone_segments = []

    for child_index, joint in enumerate(joints):
        parents = cmds.listRelatives(
            joint,
            parent=True,
            type="joint",
            fullPath=True,
        ) or []

        if parents and parents[0] in joint_index_by_name:
            bone_segments.append(
                (joint_index_by_name[parents[0]], child_index)
            )

    if not bone_segments:
        closest_index = min(
            range(len(joints)),
            key=lambda index: sum(
                (point[axis] - joint_positions[index][axis]) ** 2
                for axis in range(3)
            ),
        )
        weights = [0.0] * len(joints)
        weights[closest_index] = 1.0
        return weights

    best = None

    for parent_index, child_index in bone_segments:
        result = _point_segment_weights(
            point,
            joint_positions[parent_index],
            joint_positions[child_index],
        )

        if best is None or result[0] < best[0]:
            best = (
                result[0],
                parent_index,
                child_index,
                result[1],
                result[2],
            )

    weights = [0.0] * len(joints)
    weights[best[1]] = best[3]
    weights[best[2]] = best[4]
    return weights


def _nearest_bone_weights(node, joints):
    return _nearest_bone_weights_at_position(
        _world_position(node),
        joints,
    )


def _nearest_joint_weights(node, joints):
    """Bind one logical Curvenet node rigidly to its closest joint."""
    position = _world_position(node)
    closest_index = min(
        range(len(joints)),
        key=lambda index: sum(
            (
                position[axis] -
                _world_position(joints[index])[axis]
            ) ** 2
            for axis in range(3)
        ),
    )
    weights = [0.0] * len(joints)
    weights[closest_index] = 1.0
    return weights


def _ordered_joint_hierarchy(root_joint):
    joints = []

    def visit(joint):
        joint = cmds.ls(joint, long=True)[0]
        joints.append(joint)

        for child in cmds.listRelatives(
            joint,
            children=True,
            type="joint",
            fullPath=True,
        ) or []:
            visit(child)

    visit(root_joint)
    return joints


def _stored_joint_weights(node):
    if not cmds.attributeQuery(JOINT_WEIGHTS_ATTRIBUTE, node=node, exists=True):
        return None

    values = cmds.getAttr(node + "." + JOINT_WEIGHTS_ATTRIBUTE)

    if values and len(values) == 1 and isinstance(values[0], (list, tuple)):
        values = values[0]

    return list(values) if values else None


def _store_joint_weights(node, weights):
    if not cmds.attributeQuery(JOINT_WEIGHTS_ATTRIBUTE, node=node, exists=True):
        cmds.addAttr(
            node,
            longName=JOINT_WEIGHTS_ATTRIBUTE,
            dataType="doubleArray",
        )

    cmds.setAttr(
        node + "." + JOINT_WEIGHTS_ATTRIBUTE,
        list(weights),
        type="doubleArray",
    )


def _source_weights_for_transferred_node(node):
    logical_attribute = node + ".curvenetLogicalNodeId"

    if not cmds.objExists(logical_attribute):
        return None

    logical_node_id = cmds.getAttr(logical_attribute)

    for attribute in cmds.ls("*.curvenetLogicalNodeId", long=True) or []:
        candidate = attribute.rsplit(".", 1)[0]

        if candidate == node or not cmds.attributeQuery(
            "curvenetNode",
            node=candidate,
            exists=True,
        ):
            continue

        if cmds.getAttr(attribute) == logical_node_id:
            return _stored_joint_weights(candidate)

    return None


def _binding_weights(node, joints):
    weights = None
    copied = False

    if cmds.attributeQuery("transferredCurvenetNode", node=node, exists=True):
        weights = _source_weights_for_transferred_node(node)
        copied = weights is not None and len(weights) == len(joints)

    if weights is None or len(weights) != len(joints):
        weights = _nearest_bone_weights(node, joints)

    _store_joint_weights(node, weights)
    return weights, copied


def _logical_node_id(node):
    attribute = node + ".curvenetLogicalNodeId"

    if not cmds.objExists(attribute):
        match = re.search(r"(\d+)$", node.rsplit("|", 1)[-1])

        if not match:
            raise RuntimeError("Missing logical Curvenet node ID: " + node)

        cmds.addAttr(
            node,
            longName="curvenetLogicalNodeId",
            attributeType="long",
        )
        cmds.setAttr(attribute, int(match.group(1)))
        cmds.setAttr(attribute, lock=True)

    return cmds.getAttr(attribute)


def _curvenet_nodes_in_group(nodes_group):
    nodes = _selected_curvenet_nodes([nodes_group])

    if not nodes:
        raise RuntimeError("No Curvenet nodes found under: " + nodes_group)

    return sorted(nodes, key=_logical_node_id)


def _driver_name_for_nodes_group(nodes_group):
    short_name = nodes_group.rsplit("|", 1)[-1]

    for suffix in (
        "_drawnCurvenet_nodes_GRP",
        "_transferredCurvenet_nodes_GRP",
    ):
        if short_name.endswith(suffix):
            return short_name[:-len(suffix)] + "_curvenetWeightDriver"

    return short_name + "_curvenetWeightDriver"


def _create_curvenet_weight_driver(nodes_group, deformer):
    nodes = _curvenet_nodes_in_group(nodes_group)
    driver_name = _driver_name_for_nodes_group(nodes_group)

    if cmds.objExists(driver_name):
        cmds.delete(driver_name)

    node_positions = [_world_position(node) for node in nodes]
    minimum = [min(position[axis] for position in node_positions) for axis in range(3)]
    maximum = [max(position[axis] for position in node_positions) for axis in range(3)]
    diagonal = sum(
        (maximum[axis] - minimum[axis]) ** 2
        for axis in range(3)
    ) ** 0.5
    axis_length = max(diagonal * 0.01, 1.0e-4)
    driver_points = []

    for position in node_positions:
        driver_points.extend([
            position,
            [position[0] + axis_length, position[1], position[2]],
            [position[0], position[1] + axis_length, position[2]],
            [position[0], position[1], position[2] + axis_length],
        ])

    driver = cmds.curve(
        name=driver_name,
        degree=1,
        point=driver_points,
    )
    cmds.addAttr(
        driver,
        longName=DRIVER_MARKER_ATTRIBUTE,
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(driver + "." + DRIVER_MARKER_ATTRIBUTE, lock=True)
    cmds.addAttr(
        driver,
        longName=DRIVER_FRAME_POINT_COUNT_ATTRIBUTE,
        attributeType="long",
        defaultValue=4,
    )
    cmds.setAttr(
        driver + "." + DRIVER_FRAME_POINT_COUNT_ATTRIBUTE,
        lock=True,
    )
    for node_index, node in enumerate(nodes):
        logical_node_id = _logical_node_id(node)

        for frame_point in range(4):
            cmds.setAttr(
                f"{deformer}.inputDriverNodeIds["
                f"{node_index * 4 + frame_point}]",
                logical_node_id,
            )

    cmds.setAttr(driver + ".visibility", False)
    return driver, nodes


def bind_curvenet_driver_to_joint_hierarchy(
    root_joint,
    nodes_group,
    deformer,
    joint_influences=None,
):
    """Create and skin one paintable logical-node driver for a Curvenet."""
    root_matches = cmds.ls(root_joint, long=True, type="joint") or []

    if len(root_matches) != 1:
        raise RuntimeError("Expected exactly one root joint: " + str(root_joint))

    if not cmds.objExists(nodes_group):
        raise RuntimeError("Missing Curvenet nodes group: " + nodes_group)

    if not cmds.objExists(deformer):
        raise RuntimeError("Missing Curvenet deformer: " + deformer)

    if joint_influences is None:
        joints = _ordered_joint_hierarchy(root_matches[0])
    else:
        joints = []

        for joint in joint_influences:
            matches = cmds.ls(joint, long=True, type="joint") or []

            if len(matches) != 1:
                raise RuntimeError(
                    "Expected one joint influence: " + str(joint)
                )

            if matches[0] not in joints:
                joints.append(matches[0])

        if not joints:
            raise RuntimeError("Select at least one joint influence.")
    driver, nodes = _create_curvenet_weight_driver(nodes_group, deformer)
    copied_weight_count = 0

    for node in nodes:
        existing_constraint = _connected_constraint(node)

        if existing_constraint:
            cmds.delete(existing_constraint)

        weights, copied = _binding_weights(node, joints)
        copied_weight_count += int(copied)

    removed_curve_skins = _unskin_projected_curves(nodes)

    skin = cmds.skinCluster(
        *(joints + [driver]),
        bindMethod=0,
        skinMethod=0,
        normalizeWeights=1,
        maximumInfluences=2,
        obeyMaxInfluences=True,
        name=driver + "_skinCluster",
    )[0]
    driver = cmds.ls(driver, long=True)[0]
    driver_shape = cmds.listRelatives(
        driver,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]

    canonical_joints = {
        cmds.ls(joint, long=True)[0]: joint
        for joint in joints
    }
    actual_influences = cmds.skinCluster(
        skin,
        query=True,
        influence=True,
    ) or []
    actual_long_names = {
        cmds.ls(influence, long=True)[0]
        for influence in actual_influences
    }

    for joint_long_name, joint in canonical_joints.items():
        if joint_long_name not in actual_long_names:
            cmds.skinCluster(
                skin,
                edit=True,
                addInfluence=joint,
                weight=0.0,
                lockWeights=False,
            )

    actual_influences = cmds.skinCluster(
        skin,
        query=True,
        influence=True,
    ) or []
    influence_indices = cmds.getAttr(
        skin + ".matrix",
        multiIndices=True,
    ) or []

    if len(influence_indices) != len(actual_influences):
        raise RuntimeError(
            "Maya returned inconsistent Curvenet skin influence data."
        )

    actual_influence_data = [
        (cmds.ls(influence, long=True)[0], influence_index)
        for influence, influence_index in zip(
            actual_influences,
            influence_indices,
        )
    ]

    for node_index, node in enumerate(nodes):
        weights = _stored_joint_weights(node)

        weight_by_joint = dict(zip(canonical_joints, weights))

        for frame_point in range(4):
            cv_index = node_index * 4 + frame_point

            for influence, influence_index in actual_influence_data:
                cmds.setAttr(
                    f"{skin}.weightList[{cv_index}]"
                    f".weights[{influence_index}]",
                    weight_by_joint.get(influence, 0.0),
                )

    cmds.skinCluster(skin, edit=True, forceNormalizeWeights=True)

    cmds.connectAttr(
        driver_shape + ".worldSpace[0]",
        deformer + ".inputDriverCurve",
        force=True,
    )

    if cmds.objExists(nodes_group + ".visibility"):
        cmds.setAttr(nodes_group + ".visibility", False)

    preview_group = deformer + "_curvenet_group"

    if cmds.objExists(preview_group + ".visibility"):
        cmds.setAttr(preview_group + ".visibility", False)

    print("Curvenet driver points:", len(nodes))
    print("Copied source driver weights:", copied_weight_count)
    print("Joint influences:", len(joints))
    print("Removed legacy curve skinClusters:", removed_curve_skins)
    print("Curve evaluation: SINGLE PAINTABLE DRIVER")
    return driver, skin


def ensure_curvenet_driver_to_joint_hierarchy(
    root_joint,
    nodes_group,
    deformer,
):
    """Reuse painted source weights when a driver is already available."""
    driver_name = _driver_name_for_nodes_group(nodes_group)

    if cmds.objExists(driver_name):
        skins = cmds.ls(
            cmds.listHistory(driver_name) or [],
            type="skinCluster",
        ) or []

        if skins:
            shape = cmds.listRelatives(
                driver_name,
                shapes=True,
                noIntermediate=True,
                fullPath=True,
            )[0]

            if not cmds.isConnected(
                shape + ".worldSpace[0]",
                deformer + ".inputDriverCurve",
            ):
                cmds.connectAttr(
                    shape + ".worldSpace[0]",
                    deformer + ".inputDriverCurve",
                    force=True,
                )

            print("Reusing painted Curvenet driver:", driver_name)
            return driver_name, skins[0]

    return bind_curvenet_driver_to_joint_hierarchy(
        root_joint,
        nodes_group,
        deformer,
    )


def show_curvenet_weight_driver(mesh):
    """Show and select one Curvenet driver for weight painting."""
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    driver = prefix + "_curvenetWeightDriver"

    if not cmds.objExists(driver):
        raise RuntimeError("No Curvenet weight driver found for: " + mesh)

    cmds.setAttr(driver + ".visibility", True)
    shape = cmds.listRelatives(
        driver,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]
    cmds.select(shape + ".cv[*]", replace=True)
    print("Selected Curvenet weight driver:", driver)
    print("Use Skin > Paint Skin Weights Tool, then hide the driver when done.")
    return driver


def hide_curvenet_weight_driver(mesh):
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    driver = prefix + "_curvenetWeightDriver"

    if cmds.objExists(driver):
        cmds.setAttr(driver + ".visibility", False)


def show_curvenet_weight_controls(mesh):
    """Show the logical-node spheres used to choose editable driver CVs."""
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    groups = [
        prefix + "_drawnCurvenet_nodes_GRP",
        prefix + "_transferredCurvenet_nodes_GRP",
    ]
    nodes_group = next(
        (group for group in groups if cmds.objExists(group)),
        None,
    )

    if nodes_group is None:
        raise RuntimeError("No logical Curvenet controls found for: " + mesh)

    cmds.setAttr(nodes_group + ".visibility", True)
    print("Visible Curvenet weight controls:", nodes_group)
    print(
        "Select node spheres, then run "
        "edit_selected_curvenet_node_weights()."
    )
    return nodes_group


def hide_curvenet_weight_controls(mesh):
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]

    for suffix in (
        "_drawnCurvenet_nodes_GRP",
        "_transferredCurvenet_nodes_GRP",
    ):
        group = prefix + suffix

        if cmds.objExists(group):
            cmds.setAttr(group + ".visibility", False)


def edit_selected_curvenet_node_weights():
    """Open Maya's Component Editor for selected logical Curvenet nodes."""
    selected_nodes = _selected_curvenet_nodes(
        cmds.ls(selection=True, long=True) or []
    )

    if not selected_nodes:
        raise RuntimeError(
            "Select one or more visible Curvenet node spheres first."
        )

    parent_groups = {
        (cmds.listRelatives(node, parent=True, fullPath=True) or [None])[0]
        for node in selected_nodes
    }

    if None in parent_groups or len(parent_groups) != 1:
        raise RuntimeError(
            "Selected Curvenet nodes must belong to the same Curvenet."
        )

    nodes_group = next(iter(parent_groups))
    driver = _driver_name_for_nodes_group(nodes_group)

    if not cmds.objExists(driver):
        raise RuntimeError(
            "Bind this Curvenet to the selected joints before editing weights."
        )

    ordered_nodes = _curvenet_nodes_in_group(nodes_group)
    index_by_node = {
        cmds.ls(node, long=True)[0]: index
        for index, node in enumerate(ordered_nodes)
    }
    components = [
        f"{driver}.cv[{index_by_node[cmds.ls(node, long=True)[0]]}]"
        for node in selected_nodes
    ]

    cmds.setAttr(driver + ".visibility", True)
    cmds.select(components, replace=True)
    mel.eval("ComponentEditor;")
    print("Editing Curvenet node weights:", len(components))
    print("Use the Smooth Skins tab; each row is one selected logical node.")
    return driver, components


def paint_curvenet_driver_weights(mesh):
    """Compatibility entry point for the Curvenet-specific weight editor."""
    return open_curvenet_weight_editor(mesh)


def _curvenet_driver_and_skin(mesh):
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    driver = prefix + "_curvenetWeightDriver"

    if not cmds.objExists(driver):
        raise RuntimeError("No Curvenet weight driver found for: " + mesh)

    skins = cmds.ls(
        cmds.listHistory(driver) or [],
        type="skinCluster",
    ) or []

    if not skins:
        raise RuntimeError("The Curvenet driver is not skinned: " + driver)

    return prefix, driver, skins[0]


def _ensure_curvenet_weight_controls(mesh):
    """Create selectable markers that follow the posed driver CVs."""
    prefix, driver, skin = _curvenet_driver_and_skin(mesh)
    group = prefix + "_curvenetWeightControls_GRP"
    driver_shape = cmds.listRelatives(
        driver,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]
    cv_count = cmds.getAttr(driver_shape + ".controlPoints", size=True)
    frame_count_attribute = driver + "." + DRIVER_FRAME_POINT_COUNT_ATTRIBUTE
    frame_point_count = (
        cmds.getAttr(frame_count_attribute)
        if cmds.objExists(frame_count_attribute)
        else 1
    )
    logical_node_count = cv_count // frame_point_count
    cmds.setAttr(driver + ".visibility", False)
    bounds = cmds.exactWorldBoundingBox(driver)
    diagonal = sum(
        (bounds[index + 3] - bounds[index]) ** 2
        for index in range(3)
    ) ** 0.5
    marker_scale = max(diagonal * 0.015, 0.005)

    for suffix in (
        "_drawnCurvenet_nodes_GRP",
        "_drawnCurvenet_curves_GRP",
        "_drawnCurvenet_projectedCurves_GRP",
        "_drawnCurvenet_display_GRP",
        "_transferredCurvenet_nodes_GRP",
        "_transferredCurvenet_projectedCurves_GRP",
    ):
        neutral_group = prefix + suffix

        if cmds.objExists(neutral_group + ".visibility"):
            cmds.setAttr(neutral_group + ".visibility", False)

    if cmds.objExists(group):
        existing = cmds.listRelatives(
            group,
            children=True,
            type="transform",
            fullPath=True,
        ) or []

        if len(existing) == logical_node_count:
            for marker in existing:
                shapes = cmds.listRelatives(
                    marker,
                    shapes=True,
                    fullPath=True,
                ) or []

                for shape in shapes:
                    cmds.setAttr(shape + ".localScaleX", marker_scale)
                    cmds.setAttr(shape + ".localScaleY", marker_scale)
                    cmds.setAttr(shape + ".localScaleZ", marker_scale)

                    if cmds.attributeQuery(
                        "alwaysDrawOnTop",
                        node=shape,
                        exists=True,
                    ):
                        cmds.setAttr(shape + ".alwaysDrawOnTop", True)

            cmds.setAttr(group + ".visibility", True)
            return group, driver, skin

        cmds.delete(group)

        for point_node in cmds.ls(
            prefix + "_curvenetWeightPoint_*",
            type="pointOnCurveInfo",
        ) or []:
            cmds.delete(point_node)

    group = cmds.group(empty=True, name=group, world=True)
    for node_index in range(logical_node_count):
        marker = cmds.spaceLocator(
            name=f"{prefix}_curvenetWeightControl_{node_index}"
        )[0]
        marker = cmds.parent(marker, group)[0]
        shape = cmds.listRelatives(marker, shapes=True, fullPath=True)[0]
        cmds.setAttr(shape + ".localScaleX", marker_scale)
        cmds.setAttr(shape + ".localScaleY", marker_scale)
        cmds.setAttr(shape + ".localScaleZ", marker_scale)
        cmds.setAttr(shape + ".overrideEnabled", True)
        cmds.setAttr(shape + ".overrideColor", 17)

        if cmds.attributeQuery(
            "alwaysDrawOnTop",
            node=shape,
            exists=True,
        ):
            cmds.setAttr(shape + ".alwaysDrawOnTop", True)
        cmds.addAttr(
            marker,
            longName=WEIGHT_CONTROL_ATTRIBUTE,
            attributeType="long",
            defaultValue=node_index,
        )
        cmds.setAttr(
            marker + "." + WEIGHT_CONTROL_ATTRIBUTE,
            lock=True,
        )
        point_node = cmds.createNode(
            "pointOnCurveInfo",
            name=f"{prefix}_curvenetWeightPoint_{node_index}",
        )
        cmds.connectAttr(
            driver_shape + ".worldSpace[0]",
            point_node + ".inputCurve",
        )
        cmds.setAttr(
            point_node + ".parameter",
            node_index * frame_point_count,
        )
        cmds.connectAttr(
            point_node + ".position",
            marker + ".translate",
        )

    return group, driver, skin


def _selected_weight_control_components(driver):
    controls = []

    for selected in cmds.ls(selection=True, long=True, type="transform") or []:
        attribute = selected + "." + WEIGHT_CONTROL_ATTRIBUTE

        if cmds.objExists(attribute):
            node_index = cmds.getAttr(attribute)
            driver_shape = cmds.listRelatives(
                driver,
                shapes=True,
                noIntermediate=True,
                fullPath=True,
            )[0]
            cv_count = cmds.getAttr(
                driver_shape + ".controlPoints",
                size=True,
            )
            frame_count_attribute = (
                driver + "." + DRIVER_FRAME_POINT_COUNT_ATTRIBUTE
            )
            frame_point_count = (
                cmds.getAttr(frame_count_attribute)
                if cmds.objExists(frame_count_attribute)
                else 1
            )

            for frame_point in range(frame_point_count):
                controls.append(
                    f"{driver}.cv["
                    f"{node_index * frame_point_count + frame_point}]"
                )

    if not controls:
        raise RuntimeError(
            "Select one or more yellow Curvenet weight markers first."
        )

    return controls


def open_curvenet_weight_editor(mesh):
    """Show posed Curvenet points and edit their joint weights directly."""
    group, driver, skin = _ensure_curvenet_weight_controls(mesh)
    influences = [
        cmds.ls(influence, long=True)[0]
        for influence in (
            cmds.skinCluster(skin, query=True, influence=True) or []
        )
    ]
    base_label_by_joint = {}

    for influence in influences:
        path_parts = [part for part in influence.split("|") if part]
        short_name = path_parts[-1]
        parent_name = path_parts[-2] if len(path_parts) > 1 else "world"
        position = _world_position(influence)
        base_label_by_joint[influence] = (
            f"{short_name}  | parent: {parent_name}  | "
            f"position: {position[0]:.2f}, {position[1]:.2f}, "
            f"{position[2]:.2f}"
        )

    if cmds.window(WEIGHT_EDITOR_WINDOW, exists=True):
        cmds.deleteUI(WEIGHT_EDITOR_WINDOW)

    window = cmds.window(
        WEIGHT_EDITOR_WINDOW,
        title="Curvenet Joint Weights",
        sizeable=True,
        widthHeight=(420, 500),
    )
    layout = cmds.columnLayout(adjustableColumn=True, rowSpacing=8)
    cmds.text(
        label=(
            "Select a joint in the viewport to identify it in this list.\n"
            "Then select yellow Curvenet markers and adjust the weight."
        ),
        align="left",
    )
    status_text = cmds.text(
        label="No Curvenet markers selected.",
        align="left",
    )
    joint_list = cmds.textScrollList(
        allowMultiSelection=False,
        height=300,
    )
    show_all_field = cmds.checkBox(
        label="Show zero-weight joints (to add a new influence)",
        value=False,
    )
    weight_field = cmds.floatSliderGrp(
        label="Weight",
        field=True,
        minValue=0.0,
        maxValue=1.0,
        fieldMinValue=0.0,
        fieldMaxValue=1.0,
        value=1.0,
    )
    marker_shapes = cmds.listRelatives(
        group,
        allDescendents=True,
        type="locator",
        fullPath=True,
    ) or []
    base_marker_size = (
        cmds.getAttr(marker_shapes[0] + ".localScaleX")
        if marker_shapes
        else 1.0
    )
    marker_size_field = cmds.floatSliderGrp(
        label="Marker size",
        field=True,
        minValue=0.5,
        maxValue=5.0,
        fieldMinValue=0.1,
        fieldMaxValue=20.0,
        value=1.0,
    )
    state = {
        "components": [],
        "joint": None,
        "updating": False,
        "joint_by_label": {},
    }

    def average_weight(joint):
        if not state["components"]:
            return 0.0

        values = [
            cmds.skinPercent(
                skin,
                component,
                query=True,
                transform=joint,
            )
            for component in state["components"]
        ]
        return sum(values) / len(values)

    def refresh_joint_list(*_):
        show_all = cmds.checkBox(
            show_all_field,
            query=True,
            value=True,
        )
        selected_joint = state["joint"]
        labels = []
        state["joint_by_label"] = {}

        for influence in influences:
            weight = average_weight(influence)

            if (
                not show_all and
                weight <= 1.0e-5 and
                influence != selected_joint
            ):
                continue

            label = f"[{weight:.3f}]  {base_label_by_joint[influence]}"
            labels.append(label)
            state["joint_by_label"][label] = influence

        state["updating"] = True
        cmds.textScrollList(joint_list, edit=True, removeAll=True)

        if labels:
            cmds.textScrollList(joint_list, edit=True, append=labels)

        selected_label = next(
            (
                label
                for label, joint in state["joint_by_label"].items()
                if joint == selected_joint
            ),
            None,
        )

        if selected_label:
            cmds.textScrollList(
                joint_list,
                edit=True,
                selectItem=selected_label,
            )
            cmds.textScrollList(
                joint_list,
                edit=True,
                showIndexedItem=labels.index(selected_label) + 1,
            )

        state["updating"] = False

    def update_weight_display(*_):
        if not state["components"] or state["joint"] is None:
            return

        average = average_weight(state["joint"])
        state["updating"] = True
        cmds.floatSliderGrp(weight_field, edit=True, value=average)
        state["updating"] = False
        cmds.text(
            status_text,
            edit=True,
            label=(
                f"Selected markers: {len(state['components'])}  | "
                f"current average weight: {average:.3f}"
            ),
        )

    def choose_list_joint(*_):
        if state["updating"]:
            return

        selected_labels = cmds.textScrollList(
            joint_list,
            query=True,
            selectItem=True,
        ) or []

        if selected_labels:
            previous_joint = state["joint"]
            state["joint"] = state["joint_by_label"][selected_labels[0]]
            update_weight_display()

            if (
                previous_joint and
                previous_joint != state["joint"] and
                cmds.objExists(previous_joint)
            ):
                cmds.select(previous_joint, deselect=True)

            # Highlight the joint without replacing the selected markers.
            cmds.select(state["joint"], add=True)

    def sync_viewport_selection(*_):
        selected = cmds.ls(selection=True, long=True) or []
        marker_components = []

        for item in selected:
            attribute = item + "." + WEIGHT_CONTROL_ATTRIBUTE

            if cmds.objExists(attribute):
                node_index = cmds.getAttr(attribute)
                driver_shape = cmds.listRelatives(
                    driver,
                    shapes=True,
                    noIntermediate=True,
                    fullPath=True,
                )[0]
                cv_count = cmds.getAttr(
                    driver_shape + ".controlPoints",
                    size=True,
                )
                frame_count_attribute = (
                    driver + "." + DRIVER_FRAME_POINT_COUNT_ATTRIBUTE
                )
                frame_point_count = (
                    cmds.getAttr(frame_count_attribute)
                    if cmds.objExists(frame_count_attribute)
                    else 1
                )

                for frame_point in range(frame_point_count):
                    marker_components.append(
                        f"{driver}.cv["
                        f"{node_index * frame_point_count + frame_point}]"
                    )

        if marker_components:
            state["components"] = marker_components
            refresh_joint_list()

        selected_joints = cmds.ls(
            selection=True,
            long=True,
            type="joint",
        ) or []

        if len(selected_joints) == 1:
            selected_joint = cmds.ls(selected_joints[0], long=True)[0]

            if selected_joint in influences:
                state["joint"] = selected_joint
                refresh_joint_list()

        update_weight_display()

    def resize_markers(*_):
        multiplier = cmds.floatSliderGrp(
            marker_size_field,
            query=True,
            value=True,
        )

        for shape in marker_shapes:
            size = base_marker_size * multiplier
            cmds.setAttr(shape + ".localScaleX", size)
            cmds.setAttr(shape + ".localScaleY", size)
            cmds.setAttr(shape + ".localScaleZ", size)

    def apply_weight(*_):
        if state["updating"]:
            return

        if state["joint"] is None:
            raise RuntimeError(
                "Select a joint in the viewport or in the weight editor."
            )

        if not state["components"]:
            raise RuntimeError("Select yellow Curvenet markers first.")

        value = cmds.floatSliderGrp(
            weight_field,
            query=True,
            value=True,
        )
        cmds.skinPercent(
            skin,
            state["components"],
            transformValue=[(state["joint"], value)],
            normalize=True,
        )
        cmds.dgdirty(driver)
        cmds.refresh(force=True)
        print(
            "Set Curvenet weight:",
            len(state["components"]),
            state["joint"],
            value,
        )
        refresh_joint_list()
        update_weight_display()

    cmds.textScrollList(
        joint_list,
        edit=True,
        selectCommand=choose_list_joint,
    )
    cmds.floatSliderGrp(
        weight_field,
        edit=True,
        changeCommand=apply_weight,
    )
    cmds.checkBox(
        show_all_field,
        edit=True,
        changeCommand=refresh_joint_list,
    )
    cmds.floatSliderGrp(
        marker_size_field,
        edit=True,
        dragCommand=resize_markers,
        changeCommand=resize_markers,
    )
    cmds.button(
        label="Hide Weight Markers",
        command=lambda *_: cmds.setAttr(group + ".visibility", False),
    )
    cmds.showWindow(window)
    cmds.scriptJob(
        event=["SelectionChanged", sync_viewport_selection],
        parent=window,
    )
    sync_viewport_selection()
    print("Curvenet weight editor opened for:", mesh)
    return window


def copy_curvenet_driver_weights(source_mesh, target_mesh):
    """Copy painted logical-node weights to one transferred Curvenet."""
    source_prefix = source_mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    target_prefix = target_mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    source_driver = source_prefix + "_curvenetWeightDriver"
    target_driver = target_prefix + "_curvenetWeightDriver"

    if not cmds.objExists(source_driver) or not cmds.objExists(target_driver):
        raise RuntimeError("Both Curvenet weight drivers must exist.")

    source_skin = (cmds.ls(
        cmds.listHistory(source_driver) or [],
        type="skinCluster",
    ) or [None])[0]
    target_skin = (cmds.ls(
        cmds.listHistory(target_driver) or [],
        type="skinCluster",
    ) or [None])[0]

    if not source_skin or not target_skin:
        raise RuntimeError("Both Curvenet weight drivers must be skinned.")

    source_influences = cmds.skinCluster(
        source_skin,
        query=True,
        influence=True,
    ) or []
    target_influences = cmds.skinCluster(
        target_skin,
        query=True,
        influence=True,
    ) or []
    source_count = cmds.getAttr(
        cmds.listRelatives(source_driver, shapes=True, fullPath=True)[0]
        + ".controlPoints",
        size=True,
    )
    target_count = cmds.getAttr(
        cmds.listRelatives(target_driver, shapes=True, fullPath=True)[0]
        + ".controlPoints",
        size=True,
    )

    if source_count != target_count or len(source_influences) != len(target_influences):
        raise RuntimeError("Source and target Curvenet drivers do not correspond.")

    source_indices = cmds.getAttr(
        source_skin + ".matrix",
        multiIndices=True,
    ) or list(range(len(source_influences)))
    target_indices = cmds.getAttr(
        target_skin + ".matrix",
        multiIndices=True,
    ) or list(range(len(target_influences)))

    if len(source_indices) != len(target_indices):
        raise RuntimeError("Source and target driver influences do not correspond.")

    for cv_index in range(source_count):
        for source_index, target_index in zip(
            source_indices,
            target_indices,
        ):
            value = cmds.getAttr(
                f"{source_skin}.weightList[{cv_index}]"
                f".weights[{source_index}]"
            )
            cmds.setAttr(
                f"{target_skin}.weightList[{cv_index}]"
                f".weights[{target_index}]",
                value,
            )

    print("Copied painted Curvenet driver weights:", source_count)
    return source_count


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


def _skin_projected_curves(curves, joints, nodes):
    node_by_name = {
        node.rsplit("|", 1)[-1]: node
        for node in nodes
    }

    for curve in curves:
        existing_skin = _connected_curve_skin(curve)

        if existing_skin:
            cmds.delete(existing_skin)

        endpoint_controls = _curve_endpoint_controls(curve)
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
            weights = None

            if len(endpoint_controls) == 2:
                start_node = node_by_name.get(endpoint_controls[0])
                end_node = node_by_name.get(endpoint_controls[1])
                start_weights = (
                    _stored_joint_weights(start_node)
                    if start_node else None
                )
                end_weights = (
                    _stored_joint_weights(end_node)
                    if end_node else None
                )

                if start_weights and end_weights:
                    parameter = (
                        control_index / float(control_count - 1)
                        if control_count > 1 else 0.0
                    )
                    weights = [
                        (1.0 - parameter) * start_weight
                        + parameter * end_weight
                        for start_weight, end_weight in zip(
                            start_weights,
                            end_weights,
                        )
                    ]

            if weights is None:
                position = cmds.pointPosition(component, world=True)
                weights = _nearest_bone_weights_at_position(
                    position,
                    joints,
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


def _use_endpoint_curve_drivers(curves):
    """Keep the authored endpoint expressions for large interactive rigs."""
    driven_curves = []

    for curve in curves:
        skin = _connected_curve_skin(curve)

        if skin:
            cmds.delete(skin)

        expression = curve.rsplit("|", 1)[-1] + "_endpointExpr"

        if not cmds.objExists(expression):
            raise RuntimeError(
                "The lightweight curve driver is missing for " + curve + ". "
                "Reconnect the Curvenet in its neutral pose before binding."
            )

        driven_curves.append(curve)

    return driven_curves


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


def bind_selected_curvenet_nodes_to_joints(
    lightweight_curves=None,
):
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

    copied_weight_count = 0

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
        weights, copied = _binding_weights(node, joints)
        copied_weight_count += int(copied)
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

    projected_curves = _projected_curves_for_nodes(nodes)

    if lightweight_curves is None:
        lightweight_curves = len(projected_curves) > 64

    if lightweight_curves:
        skinned_curves = _use_endpoint_curve_drivers(projected_curves)
    else:
        skinned_curves = _skin_projected_curves(
            projected_curves,
            joints,
            nodes,
        )
    print("Bound Curvenet nodes:", len(nodes))
    print("Copied source node weights:", copied_weight_count)
    print("Skinned projected curves:", len(skinned_curves))
    print("Joint influences:", len(joints))
    print(
        "Curve evaluation:",
        "LIGHTWEIGHT ENDPOINTS" if lightweight_curves else "SKIN CLUSTERS",
    )
    return nodes


def bind_curvenet_group_to_joint_hierarchy(
    root_joint,
    nodes_group,
    lightweight_curves=None,
):
    root_matches = cmds.ls(root_joint, long=True, type="joint") or []

    if len(root_matches) != 1:
        raise RuntimeError(
            "Expected exactly one root joint: " + str(root_joint)
        )

    group_matches = cmds.ls(nodes_group, long=True, type="transform") or []

    if len(group_matches) != 1:
        raise RuntimeError(
            "Expected exactly one Curvenet nodes group: " + str(nodes_group)
        )

    root_joint = root_matches[0]
    joints = _ordered_joint_hierarchy(root_joint)

    cmds.select(joints, replace=True)
    cmds.select(group_matches[0], add=True)
    return bind_selected_curvenet_nodes_to_joints(
        lightweight_curves=lightweight_curves,
    )


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
