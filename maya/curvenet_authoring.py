"""Interactive authoring of a Curvenet on Tube A.

Load this through curvenet_workflow.py rather than executing it directly.
"""

import heapq
import math
import maya.cmds as cmds
import maya.api.OpenMaya as om
import maya.api.OpenMayaUI as omui


MESH_NAME = "tubeA"
DEFORMER_NAME = "tubeACurvenetNode"

ROOT_GRP = "tubeA_drawnCurvenet_GRP"
NODE_GRP = "tubeA_drawnCurvenet_nodes_GRP"
CURVE_GRP = "tubeA_drawnCurvenet_curves_GRP"
PROJECTED_GRP = "tubeA_drawnCurvenet_projectedCurves_GRP"
DISPLAY_GRP = "tubeA_drawnCurvenet_display_GRP"

NODE_PREFIX = "tubeA_CN_node_"
CURVE_PREFIX = "tubeA_CN_segment_"
PROJECTED_PREFIX = "tubeA_CN_projected_"
DISPLAY_PREFIX = "tubeA_CN_display_"

DRAW_CONTEXT = "tubeA_curvenetDrawContext"

SNAP_DISTANCE = 0.18
NODE_RADIUS = 0.07
PROJECTED_SAMPLES = 80

_pending_node = None
_surface_route_cache = {}

def ensure_groups():
    if not cmds.objExists(ROOT_GRP):
        cmds.group(empty=True, name=ROOT_GRP)

    for group in [
        NODE_GRP,
        CURVE_GRP,
        PROJECTED_GRP,
        DISPLAY_GRP,
    ]:
        if not cmds.objExists(group):
            cmds.group(empty=True, name=group)
            cmds.parent(group, ROOT_GRP)


def next_index(prefix):
    return len(cmds.ls(prefix + "*") or [])


def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


def mesh_shape():
    shapes = cmds.listRelatives(
        MESH_NAME,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )

    if not shapes:
        raise RuntimeError(f"No mesh shape found under {MESH_NAME}")

    return shapes[0]


def get_world_position(obj):
    return cmds.xform(
        obj,
        query=True,
        worldSpace=True,
        translation=True,
    )


def existing_nodes():
    return cmds.ls(NODE_PREFIX + "*", type="transform") or []


def _set_draw_on_top(node, enabled):
    for candidate in [node] + (
        cmds.listRelatives(node, shapes=True, fullPath=True) or []
    ):
        if cmds.attributeQuery("alwaysDrawOnTop", node=candidate, exists=True):
            cmds.setAttr(candidate + ".alwaysDrawOnTop", enabled)


def style_curvenet_node(node, pending=False):
    colour = (0.1, 1.0, 0.2) if pending else (1.0, 0.45, 0.0)

    for shape in cmds.listRelatives(node, shapes=True, fullPath=True) or []:
        cmds.setAttr(shape + ".overrideEnabled", True)
        cmds.setAttr(shape + ".overrideRGBColors", True)
        cmds.setAttr(shape + ".overrideColorRGB", *colour)

        if cmds.attributeQuery("overrideShading", node=shape, exists=True):
            cmds.setAttr(shape + ".overrideShading", False)

    _set_draw_on_top(node, True)


def style_curvenet_curve(curve):
    for shape in cmds.listRelatives(curve, shapes=True, fullPath=True) or []:
        cmds.setAttr(shape + ".overrideEnabled", True)
        cmds.setAttr(shape + ".overrideRGBColors", True)
        cmds.setAttr(shape + ".overrideColorRGB", 0.03, 0.08, 0.12)

        if cmds.attributeQuery("lineWidth", node=shape, exists=True):
            cmds.setAttr(shape + ".lineWidth", 3.5)

    _set_draw_on_top(curve, False)


def project_world_point_to_mesh(point):
    shape = mesh_shape()

    node = cmds.createNode("closestPointOnMesh")

    cmds.connectAttr(shape + ".worldMesh[0]", node + ".inMesh", force=True)
    cmds.connectAttr(shape + ".worldMatrix[0]", node + ".inputMatrix", force=True)

    cmds.setAttr(
        node + ".inPosition",
        point[0],
        point[1],
        point[2],
        type="double3",
    )

    projected = cmds.getAttr(node + ".position")[0]

    cmds.delete(node)

    return list(projected)


def project_world_point_with_normal(point):
    shape = mesh_shape()
    node = cmds.createNode("closestPointOnMesh")

    try:
        cmds.connectAttr(shape + ".worldMesh[0]", node + ".inMesh", force=True)
        cmds.connectAttr(
            shape + ".worldMatrix[0]",
            node + ".inputMatrix",
            force=True,
        )
        cmds.setAttr(
            node + ".inPosition",
            point[0],
            point[1],
            point[2],
            type="double3",
        )
        return (
            list(cmds.getAttr(node + ".position")[0]),
            list(cmds.getAttr(node + ".normal")[0]),
        )
    finally:
        cmds.delete(node)


def raycast_mesh_from_view(x, y):
    selection = om.MSelectionList()
    selection.add(MESH_NAME)

    dag_path = selection.getDagPath(0)
    dag_path.extendToShape()

    mesh_fn = om.MFnMesh(dag_path)

    view = omui.M3dView.active3dView()

    near_point = om.MPoint()
    far_point = om.MPoint()

    view.viewToWorld(
        int(x),
        int(y),
        near_point,
        far_point,
    )

    direction = far_point - near_point
    direction.normalize()

    hit = mesh_fn.closestIntersection(
        om.MFloatPoint(near_point),
        om.MFloatVector(direction),
        om.MSpace.kWorld,
        999999.0,
        False,
    )

    if hit is None:
        return None

    hit_point = hit[0]

    return [hit_point.x, hit_point.y, hit_point.z]


def find_or_create_node(position, reuse_existing=True):
    ensure_groups()

    if reuse_existing:
        for node in existing_nodes():
            node_pos = get_world_position(node)

            if distance(position, node_pos) <= SNAP_DISTANCE:
                return node

    node_id = next_index(NODE_PREFIX)

    node = cmds.polySphere(
        name=f"{NODE_PREFIX}{node_id}",
        radius=NODE_RADIUS,
        subdivisionsX=12,
        subdivisionsY=12,
    )[0]

    cmds.xform(node, worldSpace=True, translation=position)
    cmds.parent(node, NODE_GRP)

    cmds.addAttr(node, longName="curvenetNode", attributeType="bool", defaultValue=True)
    cmds.setAttr(node + ".curvenetNode", lock=True)
    style_curvenet_node(node)

    return node


def node_near_screen_position(x, y):
    view = omui.M3dView.active3dView()
    closest_node = None
    closest_distance_squared = float("inf")

    for node in existing_nodes():
        position = get_world_position(node)
        centre = view.worldToView(om.MPoint(*position))
        radius_pixels = 0.0

        for axis in range(3):
            offset = list(position)
            offset[axis] += NODE_RADIUS
            projected = view.worldToView(om.MPoint(*offset))
            radius_pixels = max(
                radius_pixels,
                math.sqrt(
                    (projected[0] - centre[0]) ** 2
                    + (projected[1] - centre[1]) ** 2
                ),
            )

        tolerance = max(12.0, radius_pixels * 1.5)
        distance_squared = (centre[0] - x) ** 2 + (centre[1] - y) ** 2

        if (
            distance_squared <= tolerance * tolerance
            and distance_squared < closest_distance_squared
        ):
            closest_node = node
            closest_distance_squared = distance_squared

    return closest_node


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


def _vector_subtract(a, b):
    return [a[index] - b[index] for index in range(3)]


def _vector_add_scaled(a, direction, scale):
    return [a[index] + direction[index] * scale for index in range(3)]


def _dot(a, b):
    return sum(a[index] * b[index] for index in range(3))


def _closest_point_on_triangle(point, a, b, c):
    """Return the closest point on one triangle."""
    ab = _vector_subtract(b, a)
    ac = _vector_subtract(c, a)
    ap = _vector_subtract(point, a)
    d1 = _dot(ab, ap)
    d2 = _dot(ac, ap)

    if d1 <= 0.0 and d2 <= 0.0:
        return a

    bp = _vector_subtract(point, b)
    d3 = _dot(ab, bp)
    d4 = _dot(ac, bp)

    if d3 >= 0.0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2

    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        return _vector_add_scaled(a, ab, d1 / (d1 - d3))

    cp = _vector_subtract(point, c)
    d5 = _dot(ab, cp)
    d6 = _dot(ac, cp)

    if d6 >= 0.0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6

    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        return _vector_add_scaled(a, ac, d2 / (d2 - d6))

    va = d3 * d6 - d5 * d4

    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        edge = _vector_subtract(c, b)
        weight = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return _vector_add_scaled(b, edge, weight)

    denominator = 1.0 / (va + vb + vc)
    v = vb * denominator
    w = vc * denominator
    return [
        a[index] + ab[index] * v + ac[index] * w
        for index in range(3)
    ]


def _surface_route_data():
    """Cache a topology-independent-enough graph with one node per face."""
    shape = mesh_shape()
    vertex_count = cmds.polyEvaluate(MESH_NAME, vertex=True)
    face_count = cmds.polyEvaluate(MESH_NAME, face=True)
    cache_key = (shape, vertex_count, face_count)

    if cache_key in _surface_route_cache:
        return _surface_route_cache[cache_key]

    selection = om.MSelectionList()
    selection.add(MESH_NAME)
    dag_path = selection.getDagPath(0)
    dag_path.extendToShape()
    mesh_fn = om.MFnMesh(dag_path)
    points = [list(point)[:3] for point in mesh_fn.getPoints(om.MSpace.kWorld)]
    faces = [list(mesh_fn.getPolygonVertices(face)) for face in range(face_count)]
    positions = list(points)
    adjacency = [[] for _ in range(vertex_count + face_count)]
    vertex_faces = [set() for _ in range(vertex_count)]

    for face_id, vertices in enumerate(faces):
        centre = [
            sum(points[vertex][axis] for vertex in vertices) / len(vertices)
            for axis in range(3)
        ]
        centre_id = vertex_count + face_id
        positions.append(centre)

        for index, vertex in enumerate(vertices):
            next_vertex = vertices[(index + 1) % len(vertices)]
            edge_length = distance(points[vertex], points[next_vertex])
            adjacency[vertex].append((next_vertex, edge_length))
            adjacency[next_vertex].append((vertex, edge_length))
            centre_length = distance(points[vertex], centre)
            adjacency[vertex].append((centre_id, centre_length))
            adjacency[centre_id].append((vertex, centre_length))
            vertex_faces[vertex].add(face_id)

    data = {
        "mesh_fn": mesh_fn,
        "points": points,
        "faces": faces,
        "positions": positions,
        "adjacency": adjacency,
        "vertex_faces": vertex_faces,
        "vertex_count": vertex_count,
    }
    _surface_route_cache.clear()
    _surface_route_cache[cache_key] = data
    return data


def _closest_mesh_face(point, mesh_fn):
    return mesh_fn.getClosestPoint(
        om.MPoint(*point),
        om.MSpace.kWorld,
    )[1]


def _short_surface_graph_path(start, end, data):
    start_face = _closest_mesh_face(start, data["mesh_fn"])
    end_face = _closest_mesh_face(end, data["mesh_fn"])
    vertex_count = data["vertex_count"]
    start_nodes = data["faces"][start_face] + [vertex_count + start_face]
    end_nodes = set(data["faces"][end_face] + [vertex_count + end_face])
    distances = {}
    previous = {}
    queue = []

    for node in start_nodes:
        cost = distance(start, data["positions"][node])
        distances[node] = cost
        heapq.heappush(queue, (cost, node))

    best_end = None
    best_total = float("inf")

    while queue:
        current_distance, node = heapq.heappop(queue)

        if current_distance != distances.get(node):
            continue

        if current_distance >= best_total:
            break

        if node in end_nodes:
            total = current_distance + distance(end, data["positions"][node])

            if total < best_total:
                best_total = total
                best_end = node

        for neighbour, edge_cost in data["adjacency"][node]:
            candidate = current_distance + edge_cost

            if candidate < distances.get(neighbour, float("inf")):
                distances[neighbour] = candidate
                previous[neighbour] = node
                heapq.heappush(queue, (candidate, neighbour))

    if best_end is None:
        return [start, end], {start_face, end_face}

    node_path = [best_end]

    while node_path[-1] in previous:
        node_path.append(previous[node_path[-1]])

    node_path.reverse()
    corridor = {start_face, end_face}

    for node in node_path:
        if node < vertex_count:
            corridor.update(data["vertex_faces"][node])
        else:
            corridor.add(node - vertex_count)

    points = [start]
    points.extend(data["positions"][node] for node in node_path)
    points.append(end)
    return points, corridor


def _project_to_face_corridor(point, corridor, data):
    best_point = None
    best_distance = float("inf")

    for face_id in corridor:
        vertices = data["faces"][face_id]
        first = data["points"][vertices[0]]

        for index in range(1, len(vertices) - 1):
            candidate = _closest_point_on_triangle(
                point,
                first,
                data["points"][vertices[index]],
                data["points"][vertices[index + 1]],
            )
            candidate_distance = distance(point, candidate)

            if candidate_distance < best_distance:
                best_distance = candidate_distance
                best_point = candidate

    return best_point if best_point is not None else point


def _smooth_surface_path(points, corridor, data, iterations=2):
    smoothed = points

    for _ in range(iterations):
        refined = [smoothed[0]]

        for first, second in zip(smoothed[:-1], smoothed[1:]):
            refined.append(lerp(first, second, 0.25))
            refined.append(lerp(first, second, 0.75))

        refined.append(smoothed[-1])
        smoothed = [
            refined[0],
            *[
                _project_to_face_corridor(point, corridor, data)
                for point in refined[1:-1]
            ],
            refined[-1],
        ]

    return smoothed


def build_short_surface_path(start, end):
    data = _surface_route_data()
    points, corridor = _short_surface_graph_path(start, end, data)
    return _smooth_surface_path(points, corridor, data)


def _normalized(vector):
    length = math.sqrt(_dot(vector, vector))

    if length <= 1.0e-10:
        return None

    return [component / length for component in vector]


def _profile_tangent(chord, normal):
    """Project the endpoint direction into its surface tangent plane."""
    projected = [
        chord[index] - normal[index] * _dot(chord, normal)
        for index in range(3)
    ]
    tangent = _normalized(projected)

    if tangent is not None:
        return tangent

    return _normalized(chord)


def create_endpoint_expression(curve, start_node, end_node):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=False,
    )[0]

    expr_name = curve + "_endpointExpr"

    if cmds.objExists(expr_name):
        cmds.delete(expr_name)

    lines = [
        f"{shape}.controlPoints[0].xValue = {start_node}.translateX;",
        f"{shape}.controlPoints[0].yValue = {start_node}.translateY;",
        f"{shape}.controlPoints[0].zValue = {start_node}.translateZ;",
        f"{shape}.controlPoints[3].xValue = {end_node}.translateX;",
        f"{shape}.controlPoints[3].yValue = {end_node}.translateY;",
        f"{shape}.controlPoints[3].zValue = {end_node}.translateZ;",
    ]

    cmds.expression(
        name=expr_name,
        string="\n".join(lines),
        alwaysEvaluate=True,
        unitConversion="all",
    )


def create_curve_between_nodes(start_node, end_node):
    ensure_groups()

    p0 = get_world_position(start_node)
    p3 = get_world_position(end_node)

    _, start_normal = project_world_point_with_normal(p0)
    _, end_normal = project_world_point_with_normal(p3)
    chord = _vector_subtract(p3, p0)
    handle_length = distance(p0, p3) / 3.0
    start_tangent = _profile_tangent(chord, start_normal)
    end_tangent = _profile_tangent(chord, end_normal)
    p1 = _vector_add_scaled(p0, start_tangent, handle_length)
    p2 = _vector_add_scaled(p3, end_tangent, -handle_length)

    curve_id = next_index(CURVE_PREFIX)

    curve = cmds.curve(
        name=f"{CURVE_PREFIX}{curve_id}",
        degree=3,
        point=[p0, p1, p2, p3],
    )

    cmds.parent(curve, CURVE_GRP)

    cmds.addAttr(curve, longName="curvenetSegment", attributeType="bool", defaultValue=True)
    cmds.setAttr(curve + ".curvenetSegment", lock=True)

    create_endpoint_expression(curve, start_node, end_node)
    create_curve_display_proxy(curve)

    print("Created Curvenet segment:", curve)

    return curve


def create_curve_display_proxy(source_curve):
    ensure_groups()
    proxy_name = DISPLAY_PREFIX + source_curve.rsplit("_", 1)[-1]

    if cmds.objExists(proxy_name):
        cmds.delete(proxy_name)

    points = []
    display_offset = NODE_RADIUS * 0.35
    closest_point = cmds.createNode("closestPointOnMesh")
    curve_info = cmds.createNode("pointOnCurveInfo")
    shape = mesh_shape()
    source_shape = cmds.listRelatives(
        source_curve,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]

    try:
        cmds.connectAttr(
            source_shape + ".worldSpace[0]",
            curve_info + ".inputCurve",
            force=True,
        )
        cmds.connectAttr(
            shape + ".worldMesh[0]",
            closest_point + ".inMesh",
            force=True,
        )
        cmds.connectAttr(
            shape + ".worldMatrix[0]",
            closest_point + ".inputMatrix",
            force=True,
        )
        minimum = cmds.getAttr(source_shape + ".minValue")
        maximum = cmds.getAttr(source_shape + ".maxValue")

        for sample_index in range(PROJECTED_SAMPLES):
            parameter = sample_index / float(PROJECTED_SAMPLES - 1)
            cmds.setAttr(
                curve_info + ".parameter",
                minimum + (maximum - minimum) * parameter,
            )
            source_point = cmds.getAttr(curve_info + ".position")[0]
            cmds.setAttr(
                closest_point + ".inPosition",
                *source_point,
                type="double3",
            )
            position = list(cmds.getAttr(closest_point + ".position")[0])
            normal = list(cmds.getAttr(closest_point + ".normal")[0])
            endpoint_taper = min(
                1.0,
                parameter * 8.0,
                (1.0 - parameter) * 8.0,
            )
            points.append([
                position[axis]
                + normal[axis] * display_offset * endpoint_taper
                for axis in range(3)
            ])
    finally:
        cmds.delete(closest_point, curve_info)

    proxy = cmds.curve(
        name=proxy_name,
        degree=1,
        point=points,
    )
    cmds.parent(proxy, DISPLAY_GRP)
    cmds.addAttr(
        proxy,
        longName="curvenetDisplayProxy",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(proxy + ".curvenetDisplayProxy", lock=True)
    style_curvenet_curve(proxy)
    cmds.setAttr(source_curve + ".visibility", False)
    return proxy


def refresh_curvenet_display():
    ensure_groups()

    for proxy in cmds.listRelatives(
        DISPLAY_GRP,
        children=True,
        type="transform",
        fullPath=True,
    ) or []:
        cmds.delete(proxy)

    proxies = []

    for curve in authored_segments():
        proxies.append(create_curve_display_proxy(curve))

    print("Refreshed Curvenet display curves:", len(proxies))
    return proxies


def on_curvenet_click():
    global _pending_node

    if _pending_node is not None and not cmds.objExists(_pending_node):
        _pending_node = None

    pos = cmds.draggerContext(
        DRAW_CONTEXT,
        query=True,
        anchorPoint=True,
    )

    hit = raycast_mesh_from_view(pos[0], pos[1])

    if hit is None:
        print("No hit on tubeA.")
        return

    node = node_near_screen_position(pos[0], pos[1])

    if node is None:
        node = find_or_create_node(hit, reuse_existing=False)

    if _pending_node is None:
        _pending_node = node
        style_curvenet_node(node, pending=True)
        print("Start node:", node)
        return

    if node == _pending_node:
        print("Clicked the same Curvenet node. Choose another point.")
        return

    start_node = _pending_node
    create_curve_between_nodes(start_node, node)
    style_curvenet_node(start_node)
    style_curvenet_node(node)
    _pending_node = None


def start_curvenet_draw_tool():
    global _pending_node

    ensure_groups()
    _pending_node = None

    if cmds.draggerContext(DRAW_CONTEXT, exists=True):
        cmds.deleteUI(DRAW_CONTEXT)

    cmds.draggerContext(
        DRAW_CONTEXT,
        pressCommand=on_curvenet_click,
        cursor="crossHair",
        space="screen",
        snapping=False,
    )

    cmds.setToolTo(DRAW_CONTEXT)

    print("Curvenet draw tool active.")
    print("Click two points on tubeA to create one segment.")


def stop_curvenet_draw_tool():
    global _pending_node

    _pending_node = None
    cmds.setToolTo("selectSuperContext")
    print("Curvenet draw tool stopped.")


def authored_segments():
    ensure_groups()

    curves = cmds.listRelatives(
        CURVE_GRP,
        children=True,
        type="transform",
        fullPath=False,
    ) or []

    return [
        curve
        for curve in curves
        if cmds.attributeQuery("curvenetSegment", node=curve, exists=True)
    ]


def point_on_curve(curve, t):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]

    info = cmds.createNode("pointOnCurveInfo")

    cmds.connectAttr(shape + ".worldSpace[0]", info + ".inputCurve", force=True)

    min_param = cmds.getAttr(shape + ".minValue")
    max_param = cmds.getAttr(shape + ".maxValue")

    param = min_param + (max_param - min_param) * t

    cmds.setAttr(info + ".parameter", param)

    pos = cmds.getAttr(info + ".position")[0]

    cmds.delete(info)

    return list(pos)


def get_curve_endpoint_controls(curve):
    expr = curve + "_endpointExpr"

    if not cmds.objExists(expr):
        raise RuntimeError(f"No endpoint expression found for {curve}.")

    expr_text = cmds.expression(expr, query=True, string=True)

    controls = []

    for line in expr_text.splitlines():
        if ".controlPoints[0].xValue" in line:
            controls.append(line.split("=")[1].strip().split(".")[0])

        if ".controlPoints[3].xValue" in line:
            controls.append(line.split("=")[1].strip().split(".")[0])

    if len(controls) != 2:
        raise RuntimeError(f"Could not read endpoint controls for {curve}.")

    return controls[0], controls[1]


def cached_projected_control_position(control, cache):
    if control in cache:
        return cache[control]

    pos = get_world_position(control)
    projected = project_world_point_to_mesh(pos)

    cache[control] = projected

    return projected


def delete_existing_projected_curves():
    ensure_groups()

    children = cmds.listRelatives(
        PROJECTED_GRP,
        children=True,
        type="transform",
        fullPath=False,
    ) or []

    if children:
        cmds.delete(children)


def create_projected_curve(curve, index, cache):
    start_control, end_control = get_curve_endpoint_controls(curve)

    points = []

    points.append(cached_projected_control_position(start_control, cache))

    for sample_index in range(1, PROJECTED_SAMPLES - 1):
        t = sample_index / float(PROJECTED_SAMPLES - 1)

        raw = point_on_curve(curve, t)
        projected = project_world_point_to_mesh(raw)

        points.append(projected)

    points.append(cached_projected_control_position(end_control, cache))

    projected_curve = cmds.curve(
        name=f"{PROJECTED_PREFIX}{index}",
        degree=1,
        point=points,
    )

    cmds.parent(projected_curve, PROJECTED_GRP)

    cmds.addAttr(
        projected_curve,
        longName="projectedCurvenetProfile",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(projected_curve + ".projectedCurvenetProfile", lock=True)

    for attribute, control in (
        ("curvenetStartControl", start_control),
        ("curvenetEndControl", end_control),
    ):
        cmds.addAttr(
            projected_curve,
            longName=attribute,
            attributeType="message",
        )
        cmds.connectAttr(
            control + ".message",
            projected_curve + "." + attribute,
            force=True,
        )

    return projected_curve


def build_projected_curves():
    ensure_groups()
    delete_existing_projected_curves()

    curves = authored_segments()

    if not curves:
        raise RuntimeError("No authored Curvenet segments found.")

    cache = {}
    projected = []

    for index, curve in enumerate(curves):
        projected_curve = create_projected_curve(curve, index, cache)
        projected.append(projected_curve)

    print("Shared projected endpoint controls:", len(cache))

    return projected


def connect_drawn_curvenet_to_plugin():
    ensure_groups()

    if cmds.objExists(DEFORMER_NAME):
        cmds.delete(DEFORMER_NAME)

    preview_group = DEFORMER_NAME + "_curvenet_group"

    if cmds.objExists(preview_group):
        cmds.delete(preview_group)

    projected_curves = build_projected_curves()

    deformer = cmds.deformer(
        MESH_NAME,
        type="curvenetNode",
        name=DEFORMER_NAME,
    )[0]

    for curve_id, curve in enumerate(projected_curves):
        shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]

        cmds.connectAttr(
            shape + ".worldSpace[0]",
            f"{deformer}.inputCurves[{curve_id}]",
            force=True,
        )

        print(f"Logical profile ID {curve_id}:", curve)

    print("\nConnected projected Curvenet to plugin.")
    print("Projected curves:", len(projected_curves))
    print("Deformer:", deformer)

    return deformer
