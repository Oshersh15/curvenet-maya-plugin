"""Public Maya workflow for authoring and transferring a Curvenet."""

import os
import sys

import maya.cmds as cmds


_WORKFLOW_FILE = globals().get("__file__", "")
_DISCOVERED_PROJECT_DIRECTORY = (
    os.path.dirname(os.path.dirname(os.path.abspath(_WORKFLOW_FILE)))
    if os.path.basename(_WORKFLOW_FILE) == "curvenet_workflow.py"
    else "/Users/osher/Desktop/BU/CAVE/MasterProject/CurvenetProject"
)
_PROJECT_DIRECTORY = os.environ.get(
    "CURVENET_PROJECT_DIR",
    _DISCOVERED_PROJECT_DIRECTORY,
)
_MAYA_DIRECTORY = _PROJECT_DIRECTORY + "/maya"
_PLUGIN_EXTENSION = (
    ".bundle" if sys.platform == "darwin"
    else (".mll" if sys.platform.startswith("win") else ".so")
)
_PLUGIN_PATH = (
    _PROJECT_DIRECTORY + "/plugin/build/CurvenetPlugin" + _PLUGIN_EXTENSION
)
_FULL_SURFACE_CURVENET = None


def _execute_project_script(filename):
    path = _MAYA_DIRECTORY + "/" + filename

    with open(path, "r") as script_file:
        source = script_file.read()

    exec(compile(source, path, "exec"), globals())


def _load_plugin():
    if not cmds.pluginInfo("CurvenetPlugin", query=True, loaded=True):
        cmds.loadPlugin(_PLUGIN_PATH)


_load_plugin()
_execute_project_script("curvenet_authoring.py")
_execute_project_script("surface_aware_authoring.py")
_execute_project_script("transfer_curvenet.py")
_execute_project_script("curvenet_joint_binding.py")


def _selected_mesh():
    selection = cmds.ls(selection=True, long=True) or []
    meshes = []

    for node in selection:
        if cmds.nodeType(node) == "mesh":
            parents = cmds.listRelatives(node, parent=True, fullPath=True) or []
            node = parents[0] if parents else node

        shapes = cmds.listRelatives(
            node,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
            type="mesh",
        ) or []

        if shapes and node not in meshes:
            meshes.append(node)

    if len(meshes) != 1:
        raise RuntimeError(
            "Select exactly one polygon mesh before starting Curvenet drawing."
        )

    return meshes[0]


def _configure_source_mesh(mesh, feature_snapping):
    global MESH_NAME, DEFORMER_NAME
    global ROOT_GRP, NODE_GRP, CURVE_GRP, PROJECTED_GRP, DISPLAY_GRP
    global NODE_PREFIX, CURVE_PREFIX, PROJECTED_PREFIX, DISPLAY_PREFIX
    global DRAW_CONTEXT
    global SNAP_DISTANCE, NODE_RADIUS
    global FEATURE_SNAP_DISTANCE, FEATURE_SNAPPING_ENABLED

    short_name = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    MESH_NAME = mesh
    DEFORMER_NAME = short_name + "CurvenetNode"
    ROOT_GRP = short_name + "_drawnCurvenet_GRP"
    NODE_GRP = short_name + "_drawnCurvenet_nodes_GRP"
    CURVE_GRP = short_name + "_drawnCurvenet_curves_GRP"
    PROJECTED_GRP = short_name + "_drawnCurvenet_projectedCurves_GRP"
    DISPLAY_GRP = short_name + "_drawnCurvenet_display_GRP"
    NODE_PREFIX = short_name + "_CN_node_"
    CURVE_PREFIX = short_name + "_CN_segment_"
    PROJECTED_PREFIX = short_name + "_CN_projected_"
    DISPLAY_PREFIX = short_name + "_CN_display_"
    DRAW_CONTEXT = short_name + "_curvenetDrawContext"
    bounds = cmds.exactWorldBoundingBox(mesh)
    diagonal = sum(
        (bounds[index + 3] - bounds[index]) ** 2
        for index in range(3)
    ) ** 0.5
    NODE_RADIUS = max(diagonal * 0.006, 1.0e-5)
    SNAP_DISTANCE = NODE_RADIUS * 1.5
    FEATURE_SNAP_DISTANCE = max(diagonal * 0.03, 1.0e-5)
    FEATURE_SNAPPING_ENABLED = feature_snapping


def start_curvenet_on_selected_mesh(
    feature_snapping=False,
    full_surface=None,
):
    """Start surface-aware Curvenet drawing on the selected mesh."""
    global _FULL_SURFACE_CURVENET

    mesh = _selected_mesh()
    _FULL_SURFACE_CURVENET = (
        None if full_surface is None else bool(full_surface)
    )
    _configure_source_mesh(mesh, feature_snapping)
    legacy_tangent_group = (
        mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
        + "_drawnCurvenet_tangents_GRP"
    )

    if cmds.objExists(legacy_tangent_group):
        cmds.delete(legacy_tangent_group)

    ensure_groups()
    cmds.setAttr(DISPLAY_GRP + ".visibility", True)
    start_curvenet_draw_tool()
    print("Drawing Curvenet on:", mesh)
    print("Feature snapping:", "ON" if feature_snapping else "OFF")
    print(
        "Curvenet coverage:",
        (
            "AUTO"
            if _FULL_SURFACE_CURVENET is None
            else (
                "FULL SURFACE"
                if _FULL_SURFACE_CURVENET
                else "AUTHORED FACES"
            )
        ),
    )
    print("Run finish_curvenet() when the Curvenet is complete.")


def clear_curvenet_on_selected_mesh():
    """Remove only the generated/authored Curvenet belonging to one mesh."""
    mesh = _selected_mesh()
    short_name = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    targets = [
        short_name + "CurvenetNode",
        short_name + "CurvenetNode_curvenet_group",
        short_name + "_drawnCurvenet_GRP",
    ]

    for target in targets:
        if cmds.objExists(target):
            cmds.delete(target)

    print("Cleared Curvenet authoring from:", mesh)


def _is_closed_surface_net():
    """Return whether every authored node continues in three directions."""
    node_degree = {}

    for curve in authored_segments():
        controls = get_curve_endpoint_controls(curve)

        if len(controls) != 2:
            return False

        for control in controls:
            control = control.rsplit("|", 1)[-1]
            node_degree[control] = node_degree.get(control, 0) + 1

    return bool(node_degree) and min(node_degree.values()) >= 3


def finish_curvenet(full_surface=None):
    """Stop drawing and construct the selected source mesh's Curvenet."""
    mesh = _selected_mesh()
    _configure_source_mesh(mesh, feature_snapping=False)
    stop_curvenet_draw_tool()
    refresh_curvenet_display()
    cmds.setAttr(DISPLAY_GRP + ".visibility", False)
    requested_coverage = (
        _FULL_SURFACE_CURVENET
        if full_surface is None
        else bool(full_surface)
    )
    resolved_full_surface = (
        _is_closed_surface_net()
        if requested_coverage is None
        else requested_coverage
    )
    print(
        "Resolved Curvenet coverage:",
        "FULL SURFACE" if resolved_full_surface else "AUTHORED FACES",
    )
    return connect_drawn_curvenet_to_plugin(
        full_surface=resolved_full_surface,
    )


def start_tube_a_curvenet():
    """Compatibility alias for selected-mesh Curvenet authoring."""
    return start_curvenet_on_selected_mesh(feature_snapping=False)


def finish_tube_a_curvenet():
    """Compatibility alias for finishing selected-mesh authoring."""
    return finish_curvenet()


def _selected_source_root():
    selected_joints = cmds.ls(
        selection=True,
        long=True,
        type="joint",
    ) or []

    if len(selected_joints) != 1:
        raise RuntimeError(
            "Select exactly one source root joint before transferring "
            "the Curvenet rig."
        )

    return selected_joints[0]


def bind_selected_curvenet_rig():
    """Bind the complete Curvenet curves to explicitly selected joints."""
    mesh = _selected_mesh()
    selected_joints = sorted(
        cmds.ls(selection=True, long=True, type="joint") or [],
        key=lambda joint: len(joint.split("|")),
    )

    if not selected_joints:
        raise RuntimeError(
            "Select the mesh and every joint that should influence it."
        )

    root_joint = selected_joints[0]
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    authored_nodes = prefix + "_drawnCurvenet_nodes_GRP"
    transferred_nodes = prefix + "_transferredCurvenet_nodes_GRP"
    nodes_group = (
        authored_nodes
        if cmds.objExists(authored_nodes)
        else transferred_nodes
    )

    if not cmds.objExists(nodes_group):
        raise RuntimeError("Missing Curvenet nodes for: " + mesh)

    deformer = prefix + "CurvenetNode"
    driver = prefix + "_curvenetWeightDriver"
    weight_controls = prefix + "_curvenetWeightControls_GRP"

    # The tube workflow skinned every projected curve CV. Keep that exact
    # deformation semantics for larger Curvenets as well; the previous
    # endpoint-only driver lost curve rotation and squashed finger caps.
    if cmds.objExists(driver):
        cmds.delete(driver)

    if cmds.objExists(weight_controls):
        cmds.delete(weight_controls)

    for point_node in cmds.ls(
        prefix + "_curvenetWeightPoint_*",
        type="pointOnCurveInfo",
    ) or []:
        cmds.delete(point_node)

    cmds.select(selected_joints, replace=True)
    cmds.select(nodes_group, add=True)
    result = bind_selected_curvenet_nodes_to_joints(
        lightweight_curves=False,
    )

    cmds.setAttr(nodes_group + ".visibility", True)
    for projected_group in (
        prefix + "_drawnCurvenet_projectedCurves_GRP",
        prefix + "_transferredCurvenet_projectedCurves_GRP",
    ):
        if cmds.objExists(projected_group):
            cmds.setAttr(projected_group + ".visibility", True)

    cmds.select(root_joint, replace=True)
    print("Bound Curvenet rig for:", mesh)
    print("Selected joint influences:", len(selected_joints))
    print("Curve evaluation: FULL CURVE SKINNING")
    return result


def setup_curvenet_rigs(
    source_mesh,
    target_mesh,
    full_surface=None,
):
    """Bind one Curvenet rig and transfer it to a second mesh."""
    source_root = _selected_source_root()
    source_prefix = source_mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    target_prefix = target_mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    source_curves_group = source_prefix + "_drawnCurvenet_curves_GRP"
    source_deformer = source_prefix + "CurvenetNode"

    if full_surface is None:
        coverage_attribute = source_deformer + ".fullSurfaceCurvenet"
        full_surface = (
            bool(cmds.getAttr(coverage_attribute))
            if cmds.objExists(coverage_attribute)
            else False
        )

    source_curves = _skinned_projected_curves_for_mesh(source_mesh)
    if not source_curves:
        raise RuntimeError(
            "Bind and finish painting the source Curvenet before transfer."
        )
    source_influences = []
    for curve in source_curves:
        skin = _connected_curve_skin(curve)
        for joint in cmds.skinCluster(
            skin,
            query=True,
            influence=True,
        ) or []:
            joint = cmds.ls(joint, long=True)[0]
            if joint not in source_influences:
                source_influences.append(joint)

    source_joints = _ordered_joint_hierarchy(source_root)
    target_root, target_joints = transfer_joint_hierarchy_to_mesh(
        source_root_joint=source_root,
        source_mesh=source_mesh,
        target_mesh=target_mesh,
        connect_pose=True,
    )
    attach_existing_curvenet_to_mesh(
        target_mesh,
        source_mesh=source_mesh,
        source_curve_group=source_curves_group,
        full_surface=full_surface,
    )
    hierarchy_map = dict(zip(source_joints, target_joints))
    missing_influences = [
        joint for joint in source_influences if joint not in hierarchy_map
    ]
    if missing_influences:
        raise RuntimeError(
            "Source curve influences are outside the transferred hierarchy."
        )
    target_influences = [
        hierarchy_map[joint]
        for joint in source_influences
    ]
    target_nodes_group = target_prefix + "_transferredCurvenet_nodes_GRP"

    # All required objects are already known here. Avoid routing this internal
    # transfer through the interactive selected-mesh command.
    cmds.select(target_influences, replace=True)
    cmds.select(target_nodes_group, add=True)
    bind_selected_curvenet_nodes_to_joints(lightweight_curves=False)
    copy_projected_curve_skin_weights(
        source_mesh,
        target_mesh,
        hierarchy_map,
    )
    _rebuild_target_deformer_from_skinned_curves(
        target_mesh,
        full_surface,
    )
    cmds.select(source_root, replace=True)
    print("Source and target Curvenet rigs are ready.")
    print("Animate the source skeleton; the target follows the same pose.")
    return target_root, target_joints


def _rebuild_target_deformer_from_skinned_curves(mesh, full_surface):
    """Connect the target plugin to the final skinned NURBS shapes."""
    prefix = mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]
    deformer_name = prefix + "CurvenetNode"
    preview_group = deformer_name + "_curvenet_group"

    if cmds.objExists(deformer_name):
        cmds.delete(deformer_name)
    if cmds.objExists(preview_group):
        cmds.delete(preview_group)

    curves = _skinned_projected_curves_for_mesh(mesh)
    curves = sorted(
        curves,
        key=lambda curve: int(
            curve.rsplit("|", 1)[-1].rsplit("_projected_", 1)[-1]
        ),
    )

    if not curves:
        raise RuntimeError("Missing skinned target Curvenet curves for: " + mesh)

    controls_by_curve = _infer_curve_endpoint_controls(curves, mesh)

    deformer = cmds.deformer(
        mesh,
        type="curvenetNode",
        name=deformer_name,
    )[0]
    linux_atomic_setup = sys.platform.startswith("linux")
    if linux_atomic_setup:
        cmds.setAttr(deformer + ".nodeState", 2)
    cmds.setAttr(deformer + ".fullSurfaceCurvenet", bool(full_surface))
    for curve_index, curve in enumerate(curves):
        shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            type="nurbsCurve",
            fullPath=True,
        )[0]
        controls = controls_by_curve.get(curve, [])
        if len(controls) != 2:
            raise RuntimeError(
                "Missing target endpoint metadata for curve: " + curve
            )
        cmds.connectAttr(
            shape + ".worldSpace[0]",
            "{}.inputCurves[{}]".format(deformer, curve_index),
            force=True,
        )
        cmds.setAttr(
            "{}.inputCurveStartNodeIds[{}]".format(deformer, curve_index),
            _logical_node_id(controls[0]),
        )
        cmds.setAttr(
            "{}.inputCurveEndNodeIds[{}]".format(deformer, curve_index),
            _logical_node_id(controls[1]),
        )

    if linux_atomic_setup:
        cmds.setAttr(deformer + ".nodeState", 0)
    cmds.dgdirty(deformer)
    cmds.refresh(force=True)
    print("Target plugin connected to skinned curves:", len(curves))
    return deformer


def setup_tube_a_and_tube_b():
    """Bind Tube A, then transfer and bind its rig and Curvenet to Tube B."""
    if not cmds.objExists("tubeB"):
        raise RuntimeError("Create Tube B before setting up the shared rig.")

    source_root = _selected_source_root()
    bind_curvenet_group_to_joint_hierarchy(
        source_root,
        "tubeA_drawnCurvenet_nodes_GRP",
    )
    target_root, target_joints = transfer_joint_hierarchy_to_mesh(
        source_root_joint=source_root,
        source_mesh="tubeA",
        target_mesh="tubeB",
        connect_pose=True,
    )
    attach_existing_curvenet_to_mesh(
        "tubeB",
        source_mesh="tubeA",
        source_curve_group="tubeA_drawnCurvenet_curves_GRP",
    )
    bind_curvenet_group_to_joint_hierarchy(
        target_root,
        "tubeB_transferredCurvenet_nodes_GRP",
    )
    cmds.select(source_root, replace=True)
    print("Tube A and Tube B Curvenet rigs are ready.")
    print("Animate the Tube A skeleton; Tube B follows the same pose.")
    return target_root, target_joints


_execute_project_script("curvenet_ui.py")


print("Curvenet workflow loaded.")
print("Select a source mesh, then run start_curvenet_on_selected_mesh().")
