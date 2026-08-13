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
CURVE_WEIGHT_EDITOR_WINDOW = "curvenetCurveWeightEditorWindow"


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
    """Show posed logical nodes and edit their joint weights directly."""
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
            "1. Select one or more yellow logical-node markers.\n"
            "2. Select a joint in the viewport or list.\n"
            "3. Adjust Weight; connected Curvenet curves follow the nodes."
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
        label="Show zero-weight joints (to add an influence)",
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
        dragCommand=apply_weight,
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


def copy_projected_curve_skin_weights(
    source_mesh,
    target_mesh,
    source_to_target_joints,
):
    """Copy full painted projected-curve CV weights between Curvenets."""
    source_curves = _skinned_projected_curves_for_mesh(source_mesh)
    target_curves = _skinned_projected_curves_for_mesh(target_mesh)

    def indexed_curves(curves):
        result = {}
        for curve in curves:
            match = re.search(r"_projected_(\d+)$", curve.rsplit("|", 1)[-1])
            if match:
                result[int(match.group(1))] = curve
        return result

    source_by_index = indexed_curves(source_curves)
    target_by_index = indexed_curves(target_curves)

    if set(source_by_index) != set(target_by_index):
        raise RuntimeError(
            "Source and target projected Curvenet curves do not correspond."
        )

    joint_map = {
        cmds.ls(source, long=True)[0]: cmds.ls(target, long=True)[0]
        for source, target in source_to_target_joints.items()
    }
    copied_cvs = 0

    for curve_index in sorted(source_by_index):
        source_curve = source_by_index[curve_index]
        target_curve = target_by_index[curve_index]
        source_skin = _connected_curve_skin(source_curve)
        target_skin = _connected_curve_skin(target_curve)

        if not source_skin or not target_skin:
            raise RuntimeError(
                "Missing projected-curve skinCluster at curve "
                + str(curve_index)
            )

        source_shape = cmds.listRelatives(
            source_curve,
            shapes=True,
            noIntermediate=True,
            type="nurbsCurve",
            fullPath=True,
        )[0]
        target_shape = cmds.listRelatives(
            target_curve,
            shapes=True,
            noIntermediate=True,
            type="nurbsCurve",
            fullPath=True,
        )[0]
        source_count = cmds.getAttr(source_shape + ".controlPoints", size=True)
        target_count = cmds.getAttr(target_shape + ".controlPoints", size=True)

        if source_count != target_count:
            raise RuntimeError(
                "Projected curve CV counts do not correspond at curve "
                + str(curve_index)
            )

        source_influences = [
            cmds.ls(joint, long=True)[0]
            for joint in (
                cmds.skinCluster(source_skin, query=True, influence=True) or []
            )
        ]

        for cv_index in range(source_count):
            source_component = source_curve + f".cv[{cv_index}]"
            target_component = target_curve + f".cv[{cv_index}]"
            target_values = []

            for source_joint in source_influences:
                if source_joint not in joint_map:
                    raise RuntimeError(
                        "No duplicated target joint for influence: "
                        + source_joint
                    )
                value = cmds.skinPercent(
                    source_skin,
                    source_component,
                    query=True,
                    transform=source_joint,
                )
                target_values.append((joint_map[source_joint], value))

            cmds.skinPercent(
                target_skin,
                target_component,
                transformValue=target_values,
                normalize=True,
            )
            copied_cvs += 1

    print("Copied painted projected-curve CV weights:", copied_cvs)
    return copied_cvs
    return source_count


def _curve_endpoint_controls(curve):
    stored_controls = []

    for attribute in ("curvenetStartControl", "curvenetEndControl"):
        if not cmds.attributeQuery(attribute, node=curve, exists=True):
            stored_controls = []
            break

        connections = cmds.listConnections(
            curve + "." + attribute,
            source=True,
            destination=False,
            type="transform",
        ) or []

        if not connections:
            stored_controls = []
            break

        stored_controls.append(connections[0].rsplit("|", 1)[-1])

    if len(stored_controls) == 2:
        return stored_controls

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


def _skinned_projected_curves_for_mesh(mesh):
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    group_names = (
        prefix + "_drawnCurvenet_projectedCurves_GRP",
        prefix + "_transferredCurvenet_projectedCurves_GRP",
    )
    curves = []

    for group_name in group_names:
        matches = cmds.ls(group_name, long=True, type="transform") or []

        for group in matches:
            for shape in cmds.listRelatives(
                group,
                allDescendents=True,
                type="nurbsCurve",
                noIntermediate=True,
                fullPath=True,
            ) or []:
                parents = cmds.listRelatives(
                    shape,
                    parent=True,
                    fullPath=True,
                ) or []

                if parents and _connected_curve_skin(parents[0]):
                    curves.append(parents[0])

    return sorted(set(curves))


def _selected_skinned_curve_cvs(curves):
    curve_names = set(curves)
    components = []

    for selected in cmds.ls(selection=True, flatten=True, long=True) or []:
        if ".cv[" in selected:
            curve = selected.split(".cv[", 1)[0]
            matches = cmds.ls(curve, long=True, type="transform") or []

            if matches and matches[0] in curve_names:
                components.append(selected)

            continue

        transforms = cmds.ls(selected, long=True, type="transform") or []

        if not transforms or transforms[0] not in curve_names:
            continue

        shape = cmds.listRelatives(
            transforms[0],
            shapes=True,
            noIntermediate=True,
            type="nurbsCurve",
            fullPath=True,
        )

        if not shape:
            continue

        cv_count = cmds.getAttr(shape[0] + ".controlPoints", size=True)
        components.extend(
            f"{transforms[0]}.cv[{cv_index}]"
            for cv_index in range(cv_count)
        )

    return sorted(set(components))


def _infer_curve_endpoint_controls(curves, mesh):
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    groups = (
        prefix + "_drawnCurvenet_nodes_GRP",
        prefix + "_transferredCurvenet_nodes_GRP",
    )
    nodes = []

    for group_name in groups:
        matches = cmds.ls(group_name, long=True, type="transform") or []

        for group in matches:
            nodes.extend(_selected_curvenet_nodes([group]))

    node_positions = {
        node.rsplit("|", 1)[-1]: _world_position(node)
        for node in nodes
    }
    controls_by_curve = {}

    for curve in curves:
        controls = _curve_endpoint_controls(curve)

        if len(controls) == 2 or not node_positions:
            controls_by_curve[curve] = controls
            continue

        shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            type="nurbsCurve",
            fullPath=True,
        )[0]
        cv_count = cmds.getAttr(shape + ".controlPoints", size=True)
        endpoints = (
            cmds.pointPosition(curve + ".cv[0]", world=True),
            cmds.pointPosition(
                curve + f".cv[{cv_count - 1}]",
                world=True,
            ),
        )
        controls_by_curve[curve] = [
            min(
                node_positions,
                key=lambda name: sum(
                    (
                        node_positions[name][axis] - endpoint[axis]
                    ) ** 2
                    for axis in range(3)
                ),
            )
            for endpoint in endpoints
        ]

    return controls_by_curve


def open_curvenet_curve_weight_editor(mesh):
    """Edit curve weights by selecting existing logical-node spheres."""
    mesh_matches = cmds.ls(mesh, long=True, type="transform") or []

    if len(mesh_matches) != 1:
        raise RuntimeError("Expected exactly one mesh transform: " + str(mesh))

    curves = _skinned_projected_curves_for_mesh(mesh_matches[0])

    if not curves:
        raise RuntimeError(
            "No fully skinned projected Curvenet curves found for " + mesh
        )

    mesh_shapes = cmds.listRelatives(
        mesh_matches[0],
        shapes=True,
        noIntermediate=True,
        type="mesh",
        fullPath=True,
    ) or []
    mesh_display_state = {}
    node_group_display_state = {}
    curve_display_state = {}

    for shape in mesh_shapes:
        mesh_display_state[shape] = (
            cmds.getAttr(shape + ".overrideEnabled"),
            cmds.getAttr(shape + ".overrideDisplayType"),
        )
        cmds.setAttr(shape + ".overrideEnabled", True)
        cmds.setAttr(shape + ".overrideDisplayType", 2)

    prefix = mesh_matches[0].rsplit("|", 1)[-1].rsplit(":", 1)[-1]

    for suffix in (
        "_drawnCurvenet_nodes_GRP",
        "_transferredCurvenet_nodes_GRP",
    ):
        group = prefix + suffix

        if cmds.objExists(group + ".visibility"):
            node_group_display_state[group] = cmds.getAttr(
                group + ".visibility"
            )
            cmds.setAttr(group + ".visibility", True)

    influences = []

    for curve in curves:
        skin = _connected_curve_skin(curve)

        for influence in cmds.skinCluster(
            skin,
            query=True,
            influence=True,
        ) or []:
            influence = cmds.ls(influence, long=True)[0]

            if influence not in influences:
                influences.append(influence)

        shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]

        curve_display_state[shape] = {
            "visibility": cmds.getAttr(shape + ".visibility"),
            "overrideEnabled": cmds.getAttr(shape + ".overrideEnabled"),
            "overrideDisplayType": cmds.getAttr(
                shape + ".overrideDisplayType"
            ),
            "lineWidth": (
                cmds.getAttr(shape + ".lineWidth")
                if cmds.objExists(shape + ".lineWidth")
                else None
            ),
            "alwaysDrawOnTop": (
                cmds.getAttr(shape + ".alwaysDrawOnTop")
                if cmds.objExists(shape + ".alwaysDrawOnTop")
                else None
            ),
        }
        cmds.setAttr(shape + ".visibility", False)
        cmds.setAttr(shape + ".overrideEnabled", True)
        cmds.setAttr(shape + ".overrideDisplayType", 2)

        if cmds.objExists(shape + ".lineWidth"):
            cmds.setAttr(shape + ".lineWidth", 3.0)

        if cmds.objExists(shape + ".alwaysDrawOnTop"):
            cmds.setAttr(shape + ".alwaysDrawOnTop", True)

    if cmds.window(CURVE_WEIGHT_EDITOR_WINDOW, exists=True):
        cmds.deleteUI(CURVE_WEIGHT_EDITOR_WINDOW)

    window = cmds.window(
        CURVE_WEIGHT_EDITOR_WINDOW,
        title="Curvenet Curve CV Weights",
        sizeable=True,
        widthHeight=(480, 560),
    )
    cmds.columnLayout(adjustableColumn=True, rowSpacing=8)
    cmds.text(
        label=(
            "1. Keep the hand posed and select one or more Curvenet spheres.\n"
            "2. Press Capture Selected Nodes and choose a joint.\n"
            "3. Adjust Weight; the change falls off along connected curves."
        ),
        align="left",
    )
    status = cmds.text(label="No curve CVs captured.", align="left")
    capture_button = cmds.button(
        label="Capture Selected Nodes"
    )
    cmds.rowLayout(numberOfColumns=2, adjustableColumn=2)
    cmds.text(label="Curve display")
    curve_display_menu = cmds.optionMenu()
    cmds.menuItem(label="Connected only")
    cmds.menuItem(label="All curves")
    cmds.menuItem(label="Hidden")
    cmds.setParent("..")
    joint_list = cmds.textScrollList(
        allowMultiSelection=False,
        height=330,
    )
    weight_field = cmds.floatSliderGrp(
        label="Weight",
        field=True,
        minValue=0.0,
        maxValue=1.0,
        fieldMinValue=0.0,
        fieldMaxValue=1.0,
        value=0.0,
    )
    state = {
        "components": [],
        "focused_curves": set(),
        "selected_controls": set(),
        "joint": None,
        "updating": False,
        "baseline_weight_vectors": {},
    }

    endpoint_controls_by_curve = _infer_curve_endpoint_controls(
        curves,
        mesh_matches[0],
    )

    def components_for_selected_nodes():
        selected_nodes = _selected_curvenet_nodes(
            cmds.ls(selection=True, long=True) or []
        )

        if not selected_nodes:
            return []

        selected_names = {
            node.rsplit("|", 1)[-1]
            for node in selected_nodes
        }
        components = []

        for curve, controls in endpoint_controls_by_curve.items():
            if len(controls) != 2:
                continue

            shape = cmds.listRelatives(
                curve,
                shapes=True,
                noIntermediate=True,
                type="nurbsCurve",
                fullPath=True,
            )[0]
            cv_count = cmds.getAttr(shape + ".controlPoints", size=True)

            if controls[0].rsplit("|", 1)[-1] in selected_names:
                components.append(curve + ".cv[0]")
            if controls[1].rsplit("|", 1)[-1] in selected_names:
                components.append(curve + f".cv[{cv_count - 1}]")

        return sorted(set(components))

    def joint_label(joint):
        parts = [part for part in joint.split("|") if part]
        short_name = parts[-1]
        parent_name = parts[-2] if len(parts) > 1 else "world"
        return short_name + "  | parent: " + parent_name

    label_to_joint = {
        joint_label(joint): joint
        for joint in influences
    }
    cmds.textScrollList(
        joint_list,
        edit=True,
        append=list(label_to_joint),
    )

    def components_by_skin():
        grouped = {}

        for component in state["components"]:
            curve = component.split(".cv[", 1)[0]
            skin = _connected_curve_skin(curve)

            if skin:
                grouped.setdefault(skin, []).append(component)

        return grouped

    def soft_component_weights():
        """Fall off across every incident edge from its selected endpoint."""
        selected_controls = state["selected_controls"]
        shared_controls = set()
        strengths = {}

        for curve, controls in endpoint_controls_by_curve.items():
            if len(controls) != 2:
                continue

            control_names = [
                control.rsplit("|", 1)[-1]
                for control in controls
            ]
            selected_start = control_names[0] in selected_controls
            selected_end = control_names[1] in selected_controls

            if not selected_start and not selected_end:
                continue

            shape = cmds.listRelatives(
                curve,
                shapes=True,
                noIntermediate=True,
                type="nurbsCurve",
                fullPath=True,
            )[0]
            cv_count = cmds.getAttr(shape + ".controlPoints", size=True)
            components = [
                curve + f".cv[{index}]"
                for index in range(cv_count)
            ]

            for index, component in enumerate(components):
                parameter = (
                    index / float(cv_count - 1)
                    if cv_count > 1
                    else 0.0
                )

                if selected_start and selected_end:
                    strength = 1.0
                elif selected_start:
                    strength = 1.0 - parameter
                else:
                    strength = parameter

                # Smoothstep gives a soft transition while keeping the
                # opposite logical node exactly unaffected.
                strengths[component] = (
                    strength * strength * (3.0 - 2.0 * strength)
                )

            if selected_start:
                shared_controls.add(controls[0])
            if selected_end:
                shared_controls.add(controls[1])

        return strengths, shared_controls

    def update_shared_spheres(shared_controls, joint, value):
        for control_name in shared_controls:
            controls = cmds.ls(control_name, long=True, type="transform") or []

            if len(controls) != 1:
                continue

            control = controls[0]
            constraint = _connected_constraint(control)

            if not constraint:
                continue

            targets = cmds.parentConstraint(
                constraint,
                query=True,
                targetList=True,
            ) or []
            aliases = cmds.parentConstraint(
                constraint,
                query=True,
                weightAliasList=True,
            ) or []
            target_data = [
                (cmds.ls(target, long=True)[0], alias)
                for target, alias in zip(targets, aliases)
            ]
            selected_alias = next(
                (
                    alias
                    for target, alias in target_data
                    if target == joint
                ),
                None,
            )

            if selected_alias is None:
                continue

            other_data = [
                (target, alias, cmds.getAttr(constraint + "." + alias))
                for target, alias in target_data
                if alias != selected_alias
            ]
            cmds.setAttr(constraint + "." + selected_alias, value)
            remaining = max(0.0, 1.0 - value)
            other_total = sum(item[2] for item in other_data)

            for _, alias, old_value in other_data:
                resolved = (
                    remaining * old_value / other_total
                    if other_total > 1.0e-8
                    else remaining / max(1, len(other_data))
                )
                cmds.setAttr(constraint + "." + alias, resolved)

            _store_joint_weights(
                control,
                [
                    cmds.getAttr(constraint + "." + alias)
                    for _, alias in target_data
                ],
            )

    def control_weight_map(control_name):
        controls = cmds.ls(control_name, long=True, type="transform") or []

        if len(controls) != 1:
            return {}

        constraint = _connected_constraint(controls[0])

        if not constraint:
            return {}

        targets = cmds.parentConstraint(
            constraint,
            query=True,
            targetList=True,
        ) or []
        aliases = cmds.parentConstraint(
            constraint,
            query=True,
            weightAliasList=True,
        ) or []
        return {
            cmds.ls(target, long=True)[0]: cmds.getAttr(
                constraint + "." + alias
            )
            for target, alias in zip(targets, aliases)
        }

    def rebuild_incident_curve_weights():
        """Derive every incident CV from its two logical-node weights."""
        for curve in state["focused_curves"]:
            controls = endpoint_controls_by_curve.get(curve, [])

            if len(controls) != 2:
                continue

            start_weights = control_weight_map(controls[0])
            end_weights = control_weight_map(controls[1])

            if not start_weights or not end_weights:
                continue

            skin = _connected_curve_skin(curve)

            if not skin:
                continue

            skin_influences = [
                cmds.ls(influence, long=True)[0]
                for influence in (
                    cmds.skinCluster(skin, query=True, influence=True) or []
                )
            ]
            shape = cmds.listRelatives(
                curve,
                shapes=True,
                noIntermediate=True,
                type="nurbsCurve",
                fullPath=True,
            )[0]
            cv_count = cmds.getAttr(shape + ".controlPoints", size=True)

            for index in range(cv_count):
                parameter = (
                    index / float(cv_count - 1)
                    if cv_count > 1
                    else 0.0
                )
                # Smooth interpolation preserves the exact endpoint weights
                # and prevents stale influences in the middle of an edge.
                blend = parameter * parameter * (3.0 - 2.0 * parameter)
                weights = [
                    (
                        (1.0 - blend) * start_weights.get(influence, 0.0) +
                        blend * end_weights.get(influence, 0.0)
                    )
                    for influence in skin_influences
                ]
                cmds.skinPercent(
                    skin,
                    curve + f".cv[{index}]",
                    transformValue=list(zip(skin_influences, weights)),
                    normalize=True,
                )

    def average_weight():
        if state["joint"] is None or not state["components"]:
            return 0.0

        values = []

        for skin, components in components_by_skin().items():
            skin_influences = {
                cmds.ls(item, long=True)[0]
                for item in (
                    cmds.skinCluster(skin, query=True, influence=True) or []
                )
            }

            if state["joint"] not in skin_influences:
                continue

            values.extend(
                cmds.skinPercent(
                    skin,
                    component,
                    query=True,
                    transform=state["joint"],
                )
                for component in components
            )

        return sum(values) / len(values) if values else 0.0

    def update_display():
        value = average_weight()
        state["updating"] = True
        cmds.floatSliderGrp(weight_field, edit=True, value=value)
        state["updating"] = False
        cmds.text(
            status,
            edit=True,
            label=(
                f"Captured CVs: {len(state['components'])}  | "
                f"average selected-joint weight: {value:.3f}"
            ),
        )

    def capture_components(*_):
        components = components_for_selected_nodes()

        if not components:
            raise RuntimeError(
                "Select one or more existing Curvenet spheres first."
            )

        state["components"] = components
        state["selected_controls"] = {
            node.rsplit("|", 1)[-1]
            for node in _selected_curvenet_nodes(
                cmds.ls(selection=True, long=True) or []
            )
        }
        state["focused_curves"] = {
            component.split(".cv[", 1)[0]
            for component in components
        }
        state["baseline_weight_vectors"] = {}
        update_curve_display()
        update_display()

    def update_curve_display(*_):
        mode = cmds.optionMenu(
            curve_display_menu,
            query=True,
            value=True,
        )

        for curve in curves:
            shapes = cmds.listRelatives(
                curve,
                shapes=True,
                noIntermediate=True,
                type="nurbsCurve",
                fullPath=True,
            ) or []

            if not shapes:
                continue

            shape = shapes[0]
            visible = (
                mode == "All curves" or
                (
                    mode == "Connected only" and
                    curve in state["focused_curves"]
                )
            )

            if cmds.objExists(shape + ".visibility"):
                cmds.setAttr(shape + ".visibility", visible)

    def choose_joint(*_):
        labels = cmds.textScrollList(
            joint_list,
            query=True,
            selectItem=True,
        ) or []

        if not labels:
            return

        state["joint"] = label_to_joint[labels[0]]
        update_display()

        # Highlighting the joint must not discard the captured CV list.
        cmds.select(state["joint"], replace=True)

    def apply_weight(*_):
        if state["updating"]:
            return

        if not state["components"]:
            raise RuntimeError("Capture existing Curvenet spheres first.")

        if state["joint"] is None:
            raise RuntimeError("Choose a joint in the list first.")

        value = cmds.floatSliderGrp(
            weight_field,
            query=True,
            value=True,
        )

        _, shared_controls = soft_component_weights()
        update_shared_spheres(shared_controls, state["joint"], value)
        rebuild_incident_curve_weights()

        cmds.refresh(force=True)
        update_display()

    cmds.button(
        capture_button,
        edit=True,
        command=capture_components,
    )
    cmds.optionMenu(
        curve_display_menu,
        edit=True,
        changeCommand=update_curve_display,
    )
    cmds.textScrollList(
        joint_list,
        edit=True,
        selectCommand=choose_joint,
    )
    cmds.floatSliderGrp(
        weight_field,
        edit=True,
        dragCommand=apply_weight,
        changeCommand=apply_weight,
    )
    def sync_selected_joint(*_):
        selected_joints = cmds.ls(
            selection=True,
            long=True,
            type="joint",
        ) or []

        if len(selected_joints) != 1 or selected_joints[0] not in influences:
            return

        state["joint"] = selected_joints[0]
        label = joint_label(state["joint"])
        cmds.textScrollList(
            joint_list,
            edit=True,
            deselectAll=True,
        )
        cmds.textScrollList(
            joint_list,
            edit=True,
            selectItem=label,
        )
        update_display()

    def restore_display(*_):
        for shape, display_state in mesh_display_state.items():
            if not cmds.objExists(shape):
                continue

            cmds.setAttr(shape + ".overrideEnabled", display_state[0])
            cmds.setAttr(shape + ".overrideDisplayType", display_state[1])

        for group, visibility in node_group_display_state.items():
            if cmds.objExists(group + ".visibility"):
                cmds.setAttr(group + ".visibility", visibility)

        for curve in curves:
            shapes = cmds.listRelatives(
                curve,
                shapes=True,
                noIntermediate=True,
                type="nurbsCurve",
                fullPath=True,
            ) or []

            if not shapes:
                continue
            display_state = curve_display_state.get(shapes[0])

            if display_state:
                cmds.setAttr(
                    shapes[0] + ".visibility",
                    display_state["visibility"],
                )
                cmds.setAttr(
                    shapes[0] + ".overrideEnabled",
                    display_state["overrideEnabled"],
                )
                cmds.setAttr(
                    shapes[0] + ".overrideDisplayType",
                    display_state["overrideDisplayType"],
                )

                if display_state["lineWidth"] is not None:
                    cmds.setAttr(
                        shapes[0] + ".lineWidth",
                        display_state["lineWidth"],
                    )

                if display_state["alwaysDrawOnTop"] is not None:
                    cmds.setAttr(
                        shapes[0] + ".alwaysDrawOnTop",
                        display_state["alwaysDrawOnTop"],
                    )

            if cmds.objExists(shapes[0] + ".dispCV"):
                cmds.setAttr(shapes[0] + ".dispCV", False)

        cmds.selectMode(object=True)
        cmds.select(clear=True)

    cmds.showWindow(window)
    cmds.scriptJob(
        event=["SelectionChanged", sync_selected_joint],
        parent=window,
    )
    cmds.scriptJob(
        uiDeleted=[window, restore_display],
        runOnce=True,
    )
    print("Curvenet curve-CV weight editor opened for:", mesh)
    return window


print("Curvenet joint binding enabled.")
