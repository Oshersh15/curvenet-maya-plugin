# Curvenet Plugin — Installation Guide

## Overview

Curvenet is a Maya C++ plugin and Python workflow for authoring a network of
profile curves on a polygon mesh, embedding that network as CutPaths, binding
the Curvenet to a skeleton, and transferring the same logical rig to a mesh
with different topology.

The public Maya interface supports:

- Surface-aware Curvenet drawing
- Partial/local and closed/wrapping Curvenets
- CutPath and Curvenet topology construction
- Curvenet-to-joint binding
- Projected-curve weight refinement
- Skeleton, Curvenet and painted-weight transfer
- Curvenet-driven mesh deformation

The polygon mesh is not directly skinned to the skeleton. Joints drive the
Curvenet, and the Curvenet drives the mesh.

---

## Requirements

Current development environment:

| Software | Version |
|---|---|
| Autodesk Maya | 2025 |
| CMake | 3.16 or newer |
| C++ standard | C++17 |
| Python | Maya 2025 embedded Python |

The current macOS build is tested. Linux and Windows build configuration is
provided but must be compiled and validated on those operating systems.

---

## Important Platform Requirement

Maya plugins are platform-specific. Compile a separate binary for every
supported operating system and Maya major version:

| Platform | Plugin file |
|---|---|
| macOS | `CurvenetPlugin.bundle` |
| Linux | `CurvenetPlugin.so` |
| Windows | `CurvenetPlugin.mll` |

---

## Build the Plugin

### macOS

```bash
cd /absolute/path/to/CurvenetProject

cmake -S plugin -B plugin/build \
  -DMAYA_LOCATION=/Applications/Autodesk/maya2025 \
  -DCMAKE_BUILD_TYPE=Release

cmake --build plugin/build
ctest --test-dir plugin/build --output-on-failure
```

### Linux

```bash
cd /absolute/path/to/CurvenetProject

cmake -S plugin -B plugin/build \
  -DMAYA_LOCATION=/usr/autodesk/maya2025 \
  -DCMAKE_BUILD_TYPE=Release

cmake --build plugin/build
ctest --test-dir plugin/build --output-on-failure
```

Change `MAYA_LOCATION` if Maya is installed elsewhere.

### Windows

Run these commands from a Visual Studio 2022 developer terminal:

```bat
cd C:\absolute\path\to\CurvenetProject

cmake -S plugin -B plugin\build ^
  -G "Visual Studio 17 2022" ^
  -A x64 ^
  -DMAYA_LOCATION="C:\Program Files\Autodesk\Maya2025"

cmake --build plugin\build --config Release
ctest --test-dir plugin\build -C Release --output-on-failure
```

GoogleTest must be discoverable by CMake when building the test target.

---

## Install Into Maya

1. Build the plugin for the current operating system.
2. Launch Maya 2025.
3. Drag `drag_to_maya.py` from the project folder into the Maya viewport.
4. The installer creates:
   - A `Curvenet.mod` Maya module file
   - A dedicated `Curvenet` shelf
   - An `Open Curvenet` launcher button
   - The required Python and plugin search paths
5. Maya selects the `Curvenet` shelf automatically.
6. Click **Open Curvenet** to open the workflow window. The installer itself
   does not open the tool window.

A restart is not required immediately after installation. Restart Maya later
to verify that the installed `.mod` module is discovered in a new session.

Users do not need to run `open_curvenet_ui()` in the Script Editor after the
shelf button has been installed.

---

## Maya Workflow

### 1. Draw Curvenet

1. Select one polygon mesh.
2. Click **Start / Continue Drawing**.
3. Draw the desired network.
4. Click **Stop Drawing** when pausing authoring.

Automatic Curvenet type detection is the normal workflow. The advanced
override can explicitly request a partial/local or closed/wrapping Curvenet.

### 2. Connect Curvenet

1. Select the authored mesh.
2. Click **Finish and Connect Plugin**.
3. Wait for the cutting and topology summary.

### 3. Bind to Joints

1. Select the mesh.
2. Shift-select every joint that should influence the Curvenet.
3. Click **Bind Curvenet to Selected Joints**.

The selected joint influences are remembered for later transfer.

### 4. Refine Weights

1. Select the bound mesh.
2. Click **Open Curve Weight Editor**.
3. Select projected Curvenet curves or CVs.
4. Capture the selection, choose a joint, and adjust its weight and soft
   radius.

### 5. Transfer to Another Mesh

1. Return the source skeleton to its neutral pose.
2. Select the original bound mesh.
3. Shift-select the target mesh.
4. Click **Transfer Complete Curvenet Rig**.

The workflow transfers the skeleton, Curvenet, logical-node weights, painted
curve-CV weights and plugin deformation setup.

---

## Cached Playback

Curvenet is a live procedural deformation system. Maya Cached Playback may
consume its memory limit when many frames are evaluated. This does not corrupt
the scene.

If necessary, open **Optional: Display & Performance** and click
**Optimize Animation Display**. This hides Curvenet viewport drawing and
disables Cached Playback while keeping deformation active.

---

## Testing

Run the C++ test suite from the project root:

```bash
ctest --test-dir plugin/build --output-on-failure
```

The current suite contains 150 tests covering HalfEdges, CutPaths, mesh
cutting, shared logical nodes, Curvenet edges and faces, face-region mapping,
and harmonic deformation.

---

## Uninstall

1. Delete the `Curvenet` shelf from Maya.
2. Delete `Curvenet.mod` from the Maya user modules directory.
3. Remove the Curvenet project folder if it is no longer needed.

Typical user module directories are located under the Maya user application
folder, for example `~/maya/modules` on macOS and Linux.
