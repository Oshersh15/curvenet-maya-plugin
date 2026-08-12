"""Single-window Maya interface for the public Curvenet workflow."""

import maya.cmds as cmds
import maya.mel as mel


CURVENET_UI_WINDOW = "curvenetWorkflowWindow"
BOUND_JOINTS_ATTRIBUTE = "curvenetBoundJointPaths"


def _ui_meshes():
    meshes = []
    for node in cmds.ls(orderedSelection=True, long=True) or []:
        if cmds.nodeType(node) == "mesh":
            parents = cmds.listRelatives(node, parent=True, fullPath=True) or []
            node = parents[0] if parents else node
        shapes = cmds.listRelatives(
            node,
            shapes=True,
            noIntermediate=True,
            type="mesh",
            fullPath=True,
        ) or []
        if shapes and node not in meshes:
            meshes.append(node)
    return meshes


def _one_ui_mesh():
    meshes = _ui_meshes()
    if len(meshes) != 1:
        raise RuntimeError("Select exactly one polygon mesh.")
    return meshes[0]


def _mesh_prefix(mesh):
    return mesh.rsplit("|", 1)[-1].rsplit(":", 1)[-1]


def _remember_bound_joints(mesh, joints):
    if not cmds.attributeQuery(BOUND_JOINTS_ATTRIBUTE, node=mesh, exists=True):
        cmds.addAttr(mesh, longName=BOUND_JOINTS_ATTRIBUTE, dataType="stringArray")
    cmds.setAttr(
        mesh + "." + BOUND_JOINTS_ATTRIBUTE,
        len(joints),
        *joints,
        type="stringArray",
    )


def _remembered_bound_joints(mesh):
    attribute = mesh + "." + BOUND_JOINTS_ATTRIBUTE
    if not cmds.objExists(attribute):
        return []
    return [
        joint
        for joint in (cmds.getAttr(attribute) or [])
        if cmds.objExists(joint)
    ]


def _run_ui_action(action):
    try:
        return action()
    except Exception as error:
        cmds.warning("Curvenet: " + str(error))
        raise


def _run_long_ui_action(label, action):
    """Run a synchronous operation with Maya's standard busy cursor."""
    try:
        cmds.waitCursor(state=True)
        return action()
    except Exception as error:
        cmds.warning("Curvenet: " + str(error))
        raise
    finally:
        cmds.waitCursor(state=False)


def _coverage_value(menu):
    label = cmds.optionMenu(menu, query=True, value=True)
    return {
        "Automatic": None,
        "Partial / local Curvenet": False,
        "Closed / wrapping Curvenet": True,
    }[label]


def _start_from_ui(feature_field, coverage_menu):
    return start_curvenet_on_selected_mesh(
        feature_snapping=cmds.checkBox(
            feature_field,
            query=True,
            value=True,
        ),
        full_surface=_coverage_value(coverage_menu),
    )


def _hide_generated_preview(mesh):
    prefix = _mesh_prefix(mesh)
    deformer = prefix + "CurvenetNode"
    generated_group = deformer + "_curvenet_group"
    show_attribute = deformer + ".showGeneratedCurvenet"

    if cmds.objExists(show_attribute):
        cmds.setAttr(show_attribute, False)
    if cmds.objExists(generated_group + ".visibility"):
        cmds.setAttr(generated_group + ".visibility", False)


def _finish_from_ui(coverage_menu):
    mesh = _one_ui_mesh()
    result = finish_curvenet(full_surface=_coverage_value(coverage_menu))
    _hide_generated_preview(mesh)
    return result


def _bind_from_ui():
    mesh = _one_ui_mesh()
    joints = sorted(
        cmds.ls(selection=True, long=True, type="joint") or [],
        key=lambda joint: len(joint.split("|")),
    )
    if not joints:
        raise RuntimeError(
            "Select the mesh, then Shift-select every joint influence."
        )
    result = bind_selected_curvenet_rig()
    _remember_bound_joints(mesh, joints)
    return result


def _paint_from_ui():
    mesh = _one_ui_mesh()
    return open_curvenet_curve_weight_editor(mesh)


def _set_curvenet_display(mesh, visible):
    prefix = _mesh_prefix(mesh)
    groups = (
        prefix + "_drawnCurvenet_nodes_GRP",
        prefix + "_drawnCurvenet_projectedCurves_GRP",
        prefix + "_transferredCurvenet_nodes_GRP",
        prefix + "_transferredCurvenet_projectedCurves_GRP",
    )
    for group in groups:
        if cmds.objExists(group + ".visibility"):
            cmds.setAttr(group + ".visibility", visible)

    if visible:
        for group in groups:
            if not cmds.objExists(group):
                continue
            for shape in cmds.listRelatives(
                group,
                allDescendents=True,
                type="nurbsCurve",
                fullPath=True,
            ) or []:
                if cmds.objExists(shape + ".dispCV"):
                    cmds.setAttr(shape + ".dispCV", False)
                if cmds.objExists(shape + ".alwaysDrawOnTop"):
                    cmds.setAttr(shape + ".alwaysDrawOnTop", False)
        cmds.selectMode(object=True)
        cmds.select(clear=True)


def disable_curvenet_cached_playback():
    """Disable Maya's memory-heavy frame cache for live Curvenet posing."""
    try:
        cmds.cacheEvaluator(edit=True, enable=False)
    except Exception:
        mel.eval("cacheEvaluator -edit -enable 0")
    print("Cached Playback disabled for Curvenet animation.")


def _animation_mode_from_ui():
    mesh = _one_ui_mesh()
    _set_curvenet_display(mesh, False)
    _hide_generated_preview(mesh)
    disable_curvenet_cached_playback()
    print("Curvenet animation display optimized for:", mesh)


def _source_root_for_transfer(source_mesh):
    remembered = _remembered_bound_joints(source_mesh)
    if remembered:
        return min(remembered, key=lambda joint: len(joint.split("|")))

    curves = _skinned_projected_curves_for_mesh(source_mesh)
    influences = []
    for curve in curves:
        skin = _connected_curve_skin(curve)
        for joint in cmds.skinCluster(
            skin,
            query=True,
            influence=True,
        ) or []:
            joint = cmds.ls(joint, long=True)[0]
            if joint not in influences:
                influences.append(joint)
    if not influences:
        raise RuntimeError("The source Curvenet has not been bound to joints.")
    return min(influences, key=lambda joint: len(joint.split("|")))


def _transfer_from_ui():
    meshes = _ui_meshes()
    if len(meshes) != 2:
        raise RuntimeError(
            "Select the original Curvenet mesh, then Shift-select the target mesh."
        )
    source_mesh, target_mesh = meshes
    source_root = _source_root_for_transfer(source_mesh)
    cmds.select(source_root, replace=True)
    return setup_curvenet_rigs(source_mesh, target_mesh)


def install_curvenet_shelf_button():
    """Install a dedicated Curvenet shelf and launcher button."""
    shelf = mel.eval("$tmpVar = $gShelfTopLevel")
    shelf_name = "Curvenet"
    existing_shelves = cmds.tabLayout(shelf, query=True, childArray=True) or []
    existing_names = {
        child.rsplit("|", 1)[-1]
        for child in existing_shelves
    }

    if shelf_name not in existing_names:
        cmds.setParent(shelf)
        cmds.shelfLayout(shelf_name, parent=shelf)
        cmds.tabLayout(
            shelf,
            edit=True,
            tabLabel=(shelf_name, shelf_name),
        )

    existing_button = None
    for child in cmds.shelfLayout(
        shelf_name,
        query=True,
        childArray=True,
    ) or []:
        if cmds.objectTypeUI(child) != "shelfButton":
            continue
        if cmds.shelfButton(child, query=True, label=True) == "Open Curvenet":
            existing_button = child
            break
    command = (
        "import os\n"
        "project = os.environ.get("
        "'CURVENET_PROJECT_DIR', {!r})\n"
        "workflow = os.path.join(project, 'maya', 'curvenet_workflow.py')\n"
        "exec(compile(open(workflow).read(), workflow, 'exec'), globals())\n"
        "open_curvenet_ui()"
    ).format(_PROJECT_DIRECTORY)
    icon_path = os.path.join(
        _PROJECT_DIRECTORY,
        "icons",
        "curvenet_icon.png",
    )
    icon = icon_path if os.path.exists(icon_path) else "commandButton.png"
    cmds.setParent(shelf_name)
    if existing_button:
        button = existing_button
        cmds.shelfButton(
            button,
            edit=True,
            command=command,
            annotation="Open the Curvenet workflow",
            sourceType="python",
            image1=icon,
        )
        print("Updated existing Open Curvenet shelf button.")
    else:
        button = cmds.shelfButton(
            parent=shelf_name,
            label="Open Curvenet",
            annotation="Open the Curvenet workflow",
            image1=icon,
            command=command,
            sourceType="python",
            docTag="CurvenetLauncher",
            width=35,
            height=35,
        )
        print("Created Open Curvenet shelf button.")
    cmds.tabLayout(shelf, edit=True, selectTab=shelf_name)
    mel.eval("saveAllShelves $gShelfTopLevel")
    print("Curvenet shelf is ready.")
    return button


def _section(title):
    frame = cmds.frameLayout(
        label=title,
        collapsable=False,
        marginWidth=10,
        marginHeight=8,
    )
    cmds.columnLayout(adjustableColumn=True, rowSpacing=6)
    return frame


def _end_section():
    cmds.setParent("..")
    cmds.setParent("..")


def open_curvenet_ui():
    """Open the complete selection-driven Curvenet workflow window."""
    cmds.selectPref(trackSelectionOrder=True)
    if cmds.window(CURVENET_UI_WINDOW, exists=True):
        cmds.deleteUI(CURVENET_UI_WINDOW)

    window = cmds.window(
        CURVENET_UI_WINDOW,
        title="Curvenet",
        sizeable=True,
        widthHeight=(430, 720),
    )
    cmds.scrollLayout(childResizable=True)
    cmds.columnLayout(adjustableColumn=True, rowSpacing=8)

    _section("1. Draw Curvenet")
    cmds.text(
        label="Select one mesh, then start drawing directly on its surface.",
        align="left",
    )
    feature_field = cmds.checkBox(
        label="Snap near hard surface features",
        value=False,
        annotation=(
            "Use only when intentionally following a rim or hard feature. "
            "Ordinary drawing remains independent of mesh vertices."
        ),
    )
    cmds.frameLayout(
        label="Advanced: Curvenet Type Override",
        collapsable=True,
        collapse=True,
        marginWidth=8,
        marginHeight=6,
    )
    cmds.columnLayout(adjustableColumn=True)
    coverage_menu = cmds.optionMenu(
        label="Coverage",
        annotation=(
            "Partial/local covers only the authored area. Closed/wrapping "
            "partitions the complete mesh. Automatic inspects the authored "
            "node connections and chooses between them."
        ),
    )
    for label in (
        "Automatic",
        "Partial / local Curvenet",
        "Closed / wrapping Curvenet",
    ):
        cmds.menuItem(label=label)
    cmds.text(
        label="Leave this on Automatic for the normal workflow.",
        align="left",
        enable=False,
    )
    cmds.setParent("..")
    cmds.setParent("..")
    cmds.button(
        label="Start / Continue Drawing",
        annotation="Select one mesh. Existing authored curves are preserved.",
        command=lambda *_: _run_ui_action(
            lambda: _start_from_ui(feature_field, coverage_menu)
        ),
    )
    cmds.button(
        label="Stop Drawing",
        command=lambda *_: _run_ui_action(stop_curvenet_draw_tool),
    )
    _end_section()

    _section("2. Connect Curvenet to Mesh")
    cmds.text(
        label="Select the authored mesh after drawing is complete.",
        align="left",
    )
    cmds.button(
        label="Finish and Connect Plugin",
        annotation=(
            "Projects the authored curves, constructs CutPaths and faces, "
            "and connects the Curvenet deformer."
        ),
        command=lambda *_: _run_long_ui_action(
            "Connecting Curvenet",
            lambda: _finish_from_ui(coverage_menu)
        ),
    )
    _end_section()

    _section("3. Skin and Refine Curvenet")
    cmds.text(
        label=(
            "Bind: select the mesh, then Shift-select every joint that should "
            "influence its Curvenet."
        ),
        align="left",
        wordWrap=True,
    )
    cmds.button(
        label="Bind Curvenet to Selected Joints",
        annotation=(
            "The mesh is not skinned. Joints drive the Curvenet; the Curvenet "
            "drives the mesh. The selected influences are remembered."
        ),
        command=lambda *_: _run_long_ui_action(
            "Binding Curvenet to joints",
            _bind_from_ui,
        ),
    )
    cmds.button(
        label="Open Curve Weight Editor",
        annotation="Select one bound mesh. Opens detailed Curvenet CV painting.",
        command=lambda *_: _run_ui_action(_paint_from_ui),
    )
    cmds.rowLayout(numberOfColumns=2, adjustableColumn=1)
    cmds.button(
        label="Show Black Curvenet",
        command=lambda *_: _run_ui_action(
            lambda: _set_curvenet_display(_one_ui_mesh(), True)
        ),
    )
    cmds.button(
        label="Hide Curvenet",
        command=lambda *_: _run_ui_action(
            lambda: _set_curvenet_display(_one_ui_mesh(), False)
        ),
    )
    cmds.setParent("..")
    _end_section()

    cmds.frameLayout(
        label="Optional: Display & Performance",
        collapsable=True,
        collapse=True,
        marginWidth=10,
        marginHeight=8,
    )
    cmds.columnLayout(adjustableColumn=True, rowSpacing=6)
    cmds.text(
        label=(
            "Use this only when Maya's Cached Playback consumes too much "
            "memory or rig drawing slows the viewport."
        ),
        align="left",
        wordWrap=True,
    )
    cmds.button(
        label="Optimize Animation Display",
        annotation=(
            "Hides Curvenet drawing and disables Maya Cached Playback. "
            "The actual deformation remains active."
        ),
        command=lambda *_: _run_ui_action(_animation_mode_from_ui),
    )
    cmds.setParent("..")
    cmds.setParent("..")

    cmds.separator(height=18, style="in")
    _section("4. Transfer to Another Mesh")
    cmds.text(
        label=(
            "Select the original bound mesh, then Shift-select the target "
            "mesh. The skeleton, Curvenet and painted weights are transferred."
        ),
        align="left",
        wordWrap=True,
    )
    cmds.button(
        label="Transfer Complete Curvenet Rig",
        annotation=(
            "Uses the remembered source joint influences. The target must not "
            "already contain a transferred Curvenet or skeleton."
        ),
        command=lambda *_: _run_long_ui_action(
            "Transferring complete Curvenet rig",
            _transfer_from_ui,
        ),
    )
    _end_section()

    cmds.text(
        label="Hover over buttons and options for additional information.",
        align="center",
        enable=False,
    )
    cmds.button(
        label="Install Curvenet Shelf Button",
        annotation=(
            "Creates a dedicated Curvenet shelf with an Open Curvenet button "
            "so this window can be launched without running a script again."
        ),
        command=lambda *_: _run_ui_action(install_curvenet_shelf_button),
    )
    cmds.showWindow(window)
    return window


print("Curvenet UI available: run open_curvenet_ui().")
