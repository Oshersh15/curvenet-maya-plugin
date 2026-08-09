"""Small public workflow for the Tube A/Tube B Curvenet demonstration.

Execute this file once in Maya's Python namespace. The public commands are:
start_tube_a_curvenet(), finish_tube_a_curvenet(), and
setup_tube_a_and_tube_b().
"""

import maya.cmds as cmds


_PROJECT_DIRECTORY = (
    "/Users/osher/Desktop/BU/CAVE/MasterProject/CurvenetProject"
)
_MAYA_DIRECTORY = _PROJECT_DIRECTORY + "/maya"
_PLUGIN_PATH = _PROJECT_DIRECTORY + "/plugin/build/CurvenetPlugin.bundle"


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


def start_tube_a_curvenet():
    """Start interactive surface-aware Curvenet drawing on Tube A."""
    if not cmds.objExists("tubeA"):
        raise RuntimeError("Create Tube A before starting Curvenet drawing.")

    start_curvenet_draw_tool()
    print("Draw the Tube A Curvenet. Run finish_tube_a_curvenet() when done.")


def finish_tube_a_curvenet():
    """Stop drawing and construct Tube A's embedded Curvenet."""
    stop_curvenet_draw_tool()
    return connect_drawn_curvenet_to_plugin()


def _selected_source_root():
    selected_joints = cmds.ls(
        selection=True,
        long=True,
        type="joint",
    ) or []

    if len(selected_joints) != 1:
        raise RuntimeError(
            "Select exactly the Tube A root joint before running "
            "setup_tube_a_and_tube_b()."
        )

    return selected_joints[0]


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


print("Curvenet workflow loaded.")
print("Next command: start_tube_a_curvenet()")
