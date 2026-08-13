"""Drag this file into Maya to install the Curvenet module and shelf button."""

import importlib
import os
import sys

import maya.cmds as cmds


def install_curvenet():
    project_directory = os.path.dirname(os.path.abspath(__file__))
    maya_scripts = os.path.join(project_directory, "maya")
    module_directory = os.path.join(
        cmds.internalVar(userAppDir=True),
        "modules",
    )
    os.makedirs(module_directory, exist_ok=True)
    module_path = os.path.join(module_directory, "Curvenet.mod")
    module_text = (
        "+ Curvenet 1.0 {project}\n"
        "PYTHONPATH +:= maya\n"
        "MAYA_PLUG_IN_PATH +:= plugin/build\n"
        "XBMLANGPATH +:= icons\n"
    ).format(project=project_directory.replace("\\", "/"))

    with open(module_path, "w") as module_file:
        module_file.write(module_text)

    cmds.loadModule(scan=True)
    try:
        cmds.loadModule(load="Curvenet")
    except RuntimeError:
        pass

    os.environ["CURVENET_PROJECT_DIR"] = project_directory
    if maya_scripts not in sys.path:
        sys.path.insert(0, maya_scripts)

    if "curvenet_workflow" in sys.modules:
        workflow = importlib.reload(sys.modules["curvenet_workflow"])
    else:
        workflow = importlib.import_module("curvenet_workflow")

    workflow.install_curvenet_shelf_button()
    return module_path


def onMayaDroppedPythonFile(*_):
    sys.modules.pop("drag_to_maya", None)
    return install_curvenet()
