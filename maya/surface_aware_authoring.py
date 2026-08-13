"""Feature-edge support for the interactive Curvenet drawing script.

Run the base drawing script first, then execute this file in the same Maya
Python namespace. Clicks near hard or boundary edges snap to that feature,
and connections between snapped nodes follow the shortest feature-edge path.
"""

import heapq
import math
import re

import maya.api.OpenMaya as om
import maya.cmds as cmds


FEATURE_SNAP_DISTANCE = 0.22
FEATURE_ANGLE_DEGREES = 45.0
FEATURE_SNAPPING_ENABLED = True
FEATURE_EDGE_ATTRIBUTE = "curvenetFeatureEdgeId"
FEATURE_EDGE_T_ATTRIBUTE = "curvenetFeatureEdgeT"


def _surface_distance(first, second):
    return math.sqrt(sum(
        (first[index] - second[index]) ** 2
        for index in range(3)
    ))


def _mesh_dag_path():
    selection = om.MSelectionList()
    selection.add(MESH_NAME)
    path = selection.getDagPath(0)
    path.extendToShape()
    return path


def _feature_graph():
    path = _mesh_dag_path()
    mesh_fn = om.MFnMesh(path)
    edge_iterator = om.MItMeshEdge(path)
    adjacency = {}
    positions = {}
    edges = {}

    while not edge_iterator.isDone():
        connected_faces = edge_iterator.getConnectedFaces()
        feature_edge = len(connected_faces) == 1

        if len(connected_faces) == 2:
            first_normal = mesh_fn.getPolygonNormal(
                connected_faces[0],
                om.MSpace.kWorld,
            )
            second_normal = mesh_fn.getPolygonNormal(
                connected_faces[1],
                om.MSpace.kWorld,
            )
            normal_dot = max(
                -1.0,
                min(1.0, first_normal * second_normal),
            )
            angle = math.degrees(math.acos(normal_dot))
            feature_edge = angle >= FEATURE_ANGLE_DEGREES

        if feature_edge:
            edge_id = edge_iterator.index()
            first = edge_iterator.vertexId(0)
            second = edge_iterator.vertexId(1)
            first_point = mesh_fn.getPoint(first, om.MSpace.kWorld)
            second_point = mesh_fn.getPoint(second, om.MSpace.kWorld)
            first_position = [first_point.x, first_point.y, first_point.z]
            second_position = [second_point.x, second_point.y, second_point.z]
            edge_length = _surface_distance(
                first_position,
                second_position,
            )

            positions[first] = first_position
            positions[second] = second_position
            edges[edge_id] = (
                first,
                second,
                first_position,
                second_position,
                edge_length,
            )
            adjacency.setdefault(first, []).append((second, edge_length))
            adjacency.setdefault(second, []).append((first, edge_length))

        edge_iterator.next()

    return adjacency, positions, edges


def _closest_point_on_segment(position, start, end):
    direction = [end[index] - start[index] for index in range(3)]
    length_squared = sum(component * component for component in direction)

    if length_squared <= 1.0e-12:
        return 0.0, list(start)

    relative = [position[index] - start[index] for index in range(3)]
    parameter = sum(
        relative[index] * direction[index]
        for index in range(3)
    ) / length_squared
    parameter = max(0.0, min(1.0, parameter))
    closest = [
        start[index] + direction[index] * parameter
        for index in range(3)
    ]
    return parameter, closest


def _nearest_feature_location(position):
    if not FEATURE_SNAPPING_ENABLED:
        return -1, 0.0, None

    _, _, edges = _feature_graph()
    best_edge = -1
    best_parameter = 0.0
    best_position = None
    best_distance = FEATURE_SNAP_DISTANCE

    for edge_id, edge in edges.items():
        parameter, closest = _closest_point_on_segment(
            position,
            edge[2],
            edge[3],
        )
        candidate_distance = _surface_distance(position, closest)

        if candidate_distance <= best_distance:
            best_edge = edge_id
            best_parameter = parameter
            best_position = closest
            best_distance = candidate_distance

    return best_edge, best_parameter, best_position


def _node_feature_location(node):
    for attribute in (FEATURE_EDGE_ATTRIBUTE, FEATURE_EDGE_T_ATTRIBUTE):
        if not cmds.attributeQuery(attribute, node=node, exists=True):
            return -1, 0.0

    return (
        cmds.getAttr(node + "." + FEATURE_EDGE_ATTRIBUTE),
        cmds.getAttr(node + "." + FEATURE_EDGE_T_ATTRIBUTE),
    )


def _shortest_vertex_path(start_vertex, end_vertex, adjacency):

    if start_vertex not in adjacency or end_vertex not in adjacency:
        return float("inf"), []

    distances = {start_vertex: 0.0}
    previous = {}
    pending = [(0.0, start_vertex)]

    while pending:
        current_distance, current_vertex = heapq.heappop(pending)

        if current_distance != distances.get(current_vertex):
            continue

        if current_vertex == end_vertex:
            break

        for neighbour, edge_length in adjacency[current_vertex]:
            candidate_distance = current_distance + edge_length

            if candidate_distance < distances.get(neighbour, float("inf")):
                distances[neighbour] = candidate_distance
                previous[neighbour] = current_vertex
                heapq.heappush(pending, (candidate_distance, neighbour))

    if end_vertex not in distances:
        return float("inf"), []

    vertex_path = [end_vertex]

    while vertex_path[-1] != start_vertex:
        vertex_path.append(previous[vertex_path[-1]])

    vertex_path.reverse()
    return distances[end_vertex], vertex_path


def _shortest_feature_path(
    start_edge_id,
    start_parameter,
    end_edge_id,
    end_parameter,
):
    adjacency, positions, edges = _feature_graph()

    if start_edge_id not in edges or end_edge_id not in edges:
        return []

    start_edge = edges[start_edge_id]
    end_edge = edges[end_edge_id]
    start_position = [
        start_edge[2][index]
        + (start_edge[3][index] - start_edge[2][index]) * start_parameter
        for index in range(3)
    ]
    end_position = [
        end_edge[2][index]
        + (end_edge[3][index] - end_edge[2][index]) * end_parameter
        for index in range(3)
    ]

    candidates = []

    if start_edge_id == end_edge_id:
        candidates.append((
            abs(start_parameter - end_parameter) * start_edge[4],
            [start_position, end_position],
        ))

    start_exits = (
        (start_edge[0], start_parameter * start_edge[4]),
        (start_edge[1], (1.0 - start_parameter) * start_edge[4]),
    )
    end_entries = (
        (end_edge[0], end_parameter * end_edge[4]),
        (end_edge[1], (1.0 - end_parameter) * end_edge[4]),
    )

    for start_vertex, start_cost in start_exits:
        for end_vertex, end_cost in end_entries:
            path_cost, vertex_ids = _shortest_vertex_path(
                start_vertex,
                end_vertex,
                adjacency,
            )

            if not vertex_ids:
                continue

            points = [start_position]
            points.extend(positions[vertex_id] for vertex_id in vertex_ids)
            points.append(end_position)
            unique_points = []

            for point in points:
                if (
                    not unique_points or
                    _surface_distance(point, unique_points[-1]) > 1.0e-8
                ):
                    unique_points.append(point)

            candidates.append((
                start_cost + path_cost + end_cost,
                unique_points,
            ))

    if not candidates:
        return []

    return min(candidates, key=lambda candidate: candidate[0])[1]


def _surface_find_or_create_node(position, reuse_existing=True):
    feature_edge, feature_parameter, feature_position = (
        _nearest_feature_location(position)
    )

    if feature_edge >= 0:
        position = feature_position

    ensure_groups()

    node = None

    if reuse_existing:
        for existing_node in existing_nodes():
            if _surface_distance(
                position,
                get_world_position(existing_node),
            ) <= SNAP_DISTANCE:
                node = existing_node
                break

    if node is None:
        node_id = next_index(NODE_PREFIX)
        node = cmds.polySphere(
            name=f"{NODE_PREFIX}{node_id}",
            radius=NODE_RADIUS,
            subdivisionsX=12,
            subdivisionsY=12,
        )[0]
        cmds.xform(node, worldSpace=True, translation=position)
        cmds.parent(node, NODE_GRP)
        cmds.addAttr(
            node,
            longName="curvenetNode",
            attributeType="bool",
            defaultValue=True,
        )
        cmds.setAttr(node + ".curvenetNode", lock=True)
        style_curvenet_node(node)

    if feature_edge >= 0:
        if not cmds.attributeQuery(FEATURE_EDGE_ATTRIBUTE, node=node, exists=True):
            cmds.addAttr(node, longName=FEATURE_EDGE_ATTRIBUTE, attributeType="long")
        if not cmds.attributeQuery(FEATURE_EDGE_T_ATTRIBUTE, node=node, exists=True):
            cmds.addAttr(node, longName=FEATURE_EDGE_T_ATTRIBUTE, attributeType="double")

        cmds.setAttr(node + "." + FEATURE_EDGE_ATTRIBUTE, lock=False)
        cmds.setAttr(node + "." + FEATURE_EDGE_T_ATTRIBUTE, lock=False)
        cmds.setAttr(node + "." + FEATURE_EDGE_ATTRIBUTE, feature_edge)
        cmds.setAttr(node + "." + FEATURE_EDGE_T_ATTRIBUTE, feature_parameter)
        cmds.setAttr(node + "." + FEATURE_EDGE_ATTRIBUTE, lock=True)
        cmds.setAttr(node + "." + FEATURE_EDGE_T_ATTRIBUTE, lock=True)

    return node


def _surface_create_endpoint_expression(curve, start_node, end_node):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=False,
    )[0]
    control_count = cmds.getAttr(shape + ".controlPoints", size=True)
    last_control = control_count - 1
    expression_name = curve + "_endpointExpr"

    if cmds.objExists(expression_name):
        cmds.delete(expression_name)

    start_rest = cmds.getAttr(start_node + ".translate")[0]
    end_rest = cmds.getAttr(end_node + ".translate")[0]
    lines = []

    # Move every curve CV by the interpolated endpoint displacement. This
    # preserves the authored edge shape instead of stretching only its first
    # or last sampled segment when a Curvenet node moves.
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
                f"({start_node}.translate{axis} - {start_rest[component]:.17g}) + "
                f"{end_weight:.17g} * "
                f"({end_node}.translate{axis} - {end_rest[component]:.17g});"
            )

    cmds.expression(
        name=expression_name,
        string="\n".join(lines),
        alwaysEvaluate=False,
        unitConversion="all",
    )


def _surface_get_curve_endpoint_controls(curve):
    expression = curve + "_endpointExpr"

    if not cmds.objExists(expression):
        raise RuntimeError(f"No endpoint expression found for {curve}.")

    expression_text = cmds.expression(expression, query=True, string=True)
    controls = []

    for line in expression_text.splitlines():
        if ".xValue" not in line or ".controlPoints[" not in line:
            continue

        for control in re.findall(r"([|:\w]+)\.translate[XYZ]", line):
            if control not in controls:
                controls.append(control)

    if len(controls) != 2:
        raise RuntimeError(f"Could not read endpoint controls for {curve}.")

    return controls[0], controls[1]


def _surface_create_curve_between_nodes(start_node, end_node):
    start_edge, start_parameter = _node_feature_location(start_node)
    end_edge, end_parameter = _node_feature_location(end_node)

    if start_edge < 0 or end_edge < 0:
        return _surface_authoring_base_create_curve(start_node, end_node)

    points = _shortest_feature_path(
        start_edge,
        start_parameter,
        end_edge,
        end_parameter,
    )

    if len(points) < 2:
        return _surface_authoring_base_create_curve(start_node, end_node)

    ensure_groups()
    curve_id = next_index(CURVE_PREFIX)
    curve = cmds.curve(
        name=f"{CURVE_PREFIX}{curve_id}",
        degree=1,
        point=points,
    )
    cmds.parent(curve, CURVE_GRP)
    cmds.addAttr(
        curve,
        longName="curvenetSegment",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(curve + ".curvenetSegment", lock=True)
    _surface_create_endpoint_expression(curve, start_node, end_node)
    style_curvenet_curve(curve)
    print("Created feature-following Curvenet segment:", curve)
    return curve


def _logical_node_id(node):
    attribute = "curvenetLogicalNodeId"

    if cmds.attributeQuery(attribute, node=node, exists=True):
        return cmds.getAttr(node + "." + attribute)

    match = re.search(r"(\d+)$", node.rsplit("|", 1)[-1])
    node_id = int(match.group(1)) if match else len(existing_nodes())
    cmds.addAttr(node, longName=attribute, attributeType="long")
    cmds.setAttr(node + "." + attribute, node_id)
    cmds.setAttr(node + "." + attribute, lock=True)
    return node_id


def _restore_deformable_mesh_shape():
    shapes = cmds.listRelatives(
        MESH_NAME,
        shapes=True,
        fullPath=True,
        type="mesh",
    ) or []
    shape_vertex_counts = []

    for shape in shapes:
        try:
            vertex_count = cmds.polyEvaluate(shape, vertex=True)
        except RuntimeError:
            vertex_count = 0

        shape_vertex_counts.append((shape, vertex_count))

        if (
            vertex_count > 0
            and not cmds.getAttr(shape + ".intermediateObject")
        ):
            return shape

    populated_shapes = [
        item for item in shape_vertex_counts if item[1] > 0
    ]

    if not populated_shapes:
        raise RuntimeError(
            f"No polygon data remains under {MESH_NAME}."
        )

    source_shape = max(populated_shapes, key=lambda item: item[1])[0]

    for shape, vertex_count in shape_vertex_counts:
        if shape != source_shape and vertex_count == 0:
            cmds.delete(shape)

    cmds.setAttr(source_shape + ".intermediateObject", False)
    cmds.setAttr(source_shape + ".visibility", True)
    return source_shape


def _deformer_source_mesh_shape(deformer):
    source_plugs = cmds.listConnections(
        deformer + ".input[0].inputGeometry",
        source=True,
        destination=False,
        plugs=True,
    ) or []

    for source_plug in source_plugs:
        source_shape = source_plug.split(".", 1)[0]

        if cmds.nodeType(source_shape) == "mesh":
            return source_shape

    raise RuntimeError(
        f"Could not find the neutral mesh connected to {deformer}."
    )


def _surface_connect_drawn_curvenet_to_plugin(full_surface=False):
    ensure_groups()

    if cmds.objExists(DEFORMER_NAME):
        cmds.delete(DEFORMER_NAME)

    _restore_deformable_mesh_shape()

    preview_group = DEFORMER_NAME + "_curvenet_group"

    if cmds.objExists(preview_group):
        cmds.delete(preview_group)

    source_curves = authored_segments()
    projected_curves = build_projected_curves()
    deformer = cmds.deformer(
        MESH_NAME,
        type="curvenetNode",
        name=DEFORMER_NAME,
    )[0]
    cmds.setAttr(
        deformer + ".fullSurfaceCurvenet",
        bool(full_surface),
    )
    source_mesh_shape = _deformer_source_mesh_shape(deformer)
    cmds.connectAttr(
        source_mesh_shape + ".outMesh",
        deformer + ".inputMesh",
        force=True,
    )

    for curve_id, (source_curve, projected_curve) in enumerate(
        zip(source_curves, projected_curves)
    ):
        start_control, end_control = get_curve_endpoint_controls(source_curve)
        _surface_create_endpoint_expression(
            projected_curve,
            start_control,
            end_control,
        )
        start_node_id = _logical_node_id(start_control)
        end_node_id = _logical_node_id(end_control)
        shape = cmds.listRelatives(
            projected_curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]
        cmds.connectAttr(
            shape + ".worldSpace[0]",
            f"{deformer}.inputCurves[{curve_id}]",
            force=True,
        )
        cmds.setAttr(
            f"{deformer}.inputCurveStartNodeIds[{curve_id}]",
            start_node_id,
        )
        cmds.setAttr(
            f"{deformer}.inputCurveEndNodeIds[{curve_id}]",
            end_node_id,
        )

    print("\nConnected projected Curvenet to plugin.")
    print("Projected curves:", len(projected_curves))
    print("Authored logical nodes:", len({
        _logical_node_id(control)
        for curve in source_curves
        for control in get_curve_endpoint_controls(curve)
    }))
    print("Deformer:", deformer)
    return deformer


# The base authoring script is executed immediately before this extension.
# Refresh these references so Maya reloads use the latest implementation.
_surface_authoring_base_find_or_create_node = find_or_create_node
_surface_authoring_base_create_curve = create_curve_between_nodes

find_or_create_node = _surface_find_or_create_node
create_endpoint_expression = _surface_create_endpoint_expression
get_curve_endpoint_controls = _surface_get_curve_endpoint_controls
create_curve_between_nodes = _surface_create_curve_between_nodes
connect_drawn_curvenet_to_plugin = _surface_connect_drawn_curvenet_to_plugin

print("Surface-aware Curvenet authoring enabled.")
print("Clicks near hard mesh features now snap and follow their edge chains.")
