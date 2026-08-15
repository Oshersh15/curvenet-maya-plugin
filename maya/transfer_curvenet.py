"""Transfer a Curvenet and joint hierarchy between corresponding Maya meshes.

Authored logical nodes, projected curves, endpoint metadata, region references,
and optional joint weights are recreated on the target surface. Projection is
continuous across neighbouring faces so curves do not jump to a nearby but
disconnected part of a complex mesh such as a hand.
"""

import re

import maya.api.OpenMaya as om
import maya.cmds as cmds


PROJECTED_SAMPLES = 80


class _ContinuousSurfaceProjector:
    """Project samples without jumping between disconnected surface faces."""

    def __init__(self, mesh):
        selection = om.MSelectionList()
        selection.add(_mesh_shape(mesh))
        self.dag_path = selection.getDagPath(0)
        self.mesh_fn = om.MFnMesh(self.dag_path)
        self.face_iterator = om.MItMeshPolygon(self.dag_path)
        self.neighbours = {}
        self.face_vertices = {}

    def closest_point_and_face(self, point):
        projected, face_id = self.mesh_fn.getClosestPoint(
            om.MPoint(point),
            om.MSpace.kWorld,
        )
        return list(projected)[:3], int(face_id)

    def _face_neighbours(self, face_id):
        if face_id not in self.neighbours:
            self.face_iterator.setIndex(face_id)
            self.neighbours[face_id] = list(
                self.face_iterator.getConnectedFaces()
            )
        return self.neighbours[face_id]

    def _face_vertex_ids(self, face_id):
        if face_id not in self.face_vertices:
            self.face_iterator.setIndex(face_id)
            self.face_vertices[face_id] = list(
                self.face_iterator.getVertices()
            )
        return self.face_vertices[face_id]

    def _shared_edge_point(self, first_face, second_face, target):
        shared_vertices = list(
            set(self._face_vertex_ids(first_face)).intersection(
                self._face_vertex_ids(second_face)
            )
        )

        if len(shared_vertices) != 2:
            raise RuntimeError(
                "Adjacent target faces do not share exactly one mesh edge."
            )

        first = self.mesh_fn.getPoint(
            shared_vertices[0], om.MSpace.kWorld
        )
        second = self.mesh_fn.getPoint(
            shared_vertices[1], om.MSpace.kWorld
        )
        edge = om.MVector(second) - om.MVector(first)
        length_squared = edge * edge

        if length_squared <= 1.0e-20:
            return list(first)[:3]

        parameter = (
            edge * (om.MVector(target) - om.MVector(first))
        ) / length_squared
        parameter = max(0.0, min(1.0, parameter))
        crossing = om.MVector(first) + edge * parameter
        return [crossing.x, crossing.y, crossing.z]

    def _surface_points(self, points, faces):
        result = [points[0]]

        for index in range(1, len(points)):
            if faces[index] != faces[index - 1]:
                midpoint = [
                    0.5 * (points[index - 1][axis] + points[index][axis])
                    for axis in range(3)
                ]
                result.append(
                    self._shared_edge_point(
                        faces[index - 1],
                        faces[index],
                        midpoint,
                    )
                )

            result.append(points[index])

        return result

    @staticmethod
    def _closest_point_on_triangle(point, first, second, third):
        """Return the closest point on one triangle (Ericson regions)."""
        point = om.MVector(point)
        first = om.MVector(first)
        second = om.MVector(second)
        third = om.MVector(third)
        first_second = second - first
        first_third = third - first
        first_point = point - first
        d1 = first_second * first_point
        d2 = first_third * first_point

        if d1 <= 0.0 and d2 <= 0.0:
            return first

        second_point = point - second
        d3 = first_second * second_point
        d4 = first_third * second_point

        if d3 >= 0.0 and d4 <= d3:
            return second

        vc = d1 * d4 - d3 * d2
        if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
            weight = d1 / (d1 - d3)
            return first + first_second * weight

        third_point = point - third
        d5 = first_second * third_point
        d6 = first_third * third_point

        if d6 >= 0.0 and d5 <= d6:
            return third

        vb = d5 * d2 - d1 * d6
        if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
            weight = d2 / (d2 - d6)
            return first + first_third * weight

        va = d3 * d6 - d5 * d4
        if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
            edge = third - second
            weight = (d4 - d3) / ((d4 - d3) + (d5 - d6))
            return second + edge * weight

        denominator = 1.0 / (va + vb + vc)
        second_weight = vb * denominator
        third_weight = vc * denominator
        return (
            first
            + first_second * second_weight
            + first_third * third_weight
        )

    def _point_on_face(self, face_id, point):
        self.face_iterator.setIndex(face_id)
        triangle_points, _ = self.face_iterator.getTriangles(
            om.MSpace.kWorld,
        )
        closest = None
        closest_distance = float("inf")

        for index in range(0, len(triangle_points), 3):
            candidate = self._closest_point_on_triangle(
                point,
                triangle_points[index],
                triangle_points[index + 1],
                triangle_points[index + 2],
            )
            squared_distance = sum(
                (candidate[axis] - point[axis]) ** 2
                for axis in range(3)
            )

            if squared_distance < closest_distance:
                closest = candidate
                closest_distance = squared_distance

        if closest is None:
            raise RuntimeError(
                f"Target mesh face {face_id} has no triangles."
            )

        return [closest.x, closest.y, closest.z]

    def _distance_to_face(self, start_face, end_face):
        distances = {end_face: 0}
        pending = [end_face]

        for face_id in pending:
            next_distance = distances[face_id] + 1
            for neighbour in self._face_neighbours(face_id):
                if neighbour not in distances:
                    distances[neighbour] = next_distance
                    pending.append(neighbour)

        return distances.get(start_face), distances

    def project_polyline(self, targets, start_point, end_point):
        if len(targets) < 2:
            return [start_point, end_point]

        _, start_face = self.closest_point_and_face(start_point)
        _, end_face = self.closest_point_and_face(end_point)
        face_distance, distance_to_end = self._distance_to_face(
            start_face,
            end_face,
        )

        if face_distance is None or face_distance > len(targets) - 1:
            raise RuntimeError(
                "Transferred Curvenet edge needs more surface samples "
                f"({face_distance} face steps for {len(targets) - 1} intervals)."
            )

        # Dynamic programming chooses the closest surface samples while only
        # allowing the curve to remain in a face or enter an adjacent face.
        states = {
            start_face: (0.0, [list(start_point)], [start_face])
        }

        for sample_index in range(1, len(targets) - 1):
            target = targets[sample_index]
            remaining_steps = len(targets) - 1 - sample_index
            next_states = {}

            for face_id, (cost, points, faces) in states.items():
                candidates = [face_id] + self._face_neighbours(face_id)

                for candidate in candidates:
                    if distance_to_end.get(candidate, 10 ** 9) > remaining_steps:
                        continue

                    projected = self._point_on_face(candidate, target)
                    squared_distance = sum(
                        (projected[axis] - target[axis]) ** 2
                        for axis in range(3)
                    )
                    candidate_cost = cost + squared_distance
                    existing = next_states.get(candidate)

                    if existing is None or candidate_cost < existing[0]:
                        next_states[candidate] = (
                            candidate_cost,
                            points + [projected],
                            faces + [candidate],
                        )

            if not next_states:
                raise RuntimeError(
                    "Could not project a continuous Curvenet edge onto the target mesh."
                )

            # Keep the search bounded on dense production meshes.
            states = dict(
                sorted(next_states.items(), key=lambda item: item[1][0])[:64]
            )

        final_candidates = []
        for face_id, (cost, points, faces) in states.items():
            if face_id == end_face or end_face in self._face_neighbours(face_id):
                final_candidates.append((cost, points, faces))

        if not final_candidates:
            raise RuntimeError(
                "Transferred Curvenet edge did not reach its logical endpoint continuously."
            )

        _, best_points, best_faces = min(
            final_candidates,
            key=lambda item: item[0],
        )
        best_points.append(list(end_point))
        best_faces.append(end_face)
        return self._surface_points(best_points, best_faces)


def _mesh_shape(mesh):
    shapes = cmds.listRelatives(
        mesh,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    ) or []

    if not shapes:
        raise RuntimeError(f"No mesh shape found under {mesh}.")

    return shapes[0]


def _short_name(node):
    return node.rsplit("|", 1)[-1].replace(":", "_")


def _point_on_curve(curve, parameter):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=True,
    )[0]
    info = cmds.createNode("pointOnCurveInfo")

    try:
        cmds.connectAttr(
            shape + ".worldSpace[0]",
            info + ".inputCurve",
            force=True,
        )
        minimum = cmds.getAttr(shape + ".minValue")
        maximum = cmds.getAttr(shape + ".maxValue")
        value = minimum + (maximum - minimum) * parameter
        cmds.setAttr(info + ".parameter", value)
        return list(cmds.getAttr(info + ".position")[0])
    finally:
        cmds.delete(info)


def _endpoint_controls(curve):
    expression = curve + "_endpointExpr"

    if not cmds.objExists(expression):
        raise RuntimeError(
            f"No endpoint expression found for source curve {curve}."
        )

    expression_text = cmds.expression(
        expression,
        query=True,
        string=True,
    )
    controls = []

    for line in expression_text.splitlines():
        if ".xValue" not in line or ".controlPoints[" not in line:
            continue

        for control in re.findall(r"([|:\w]+)\.translate[XYZ]", line):
            if control not in controls:
                controls.append(control)

    if len(controls) != 2:
        raise RuntimeError(
            f"Could not identify both endpoint controls for {curve}."
        )

    return controls[0], controls[1]


def _authored_segments(source_curve_group):
    if not cmds.objExists(source_curve_group):
        raise RuntimeError(
            f"Source Curvenet group does not exist: {source_curve_group}"
        )

    curves = cmds.listRelatives(
        source_curve_group,
        children=True,
        type="transform",
        fullPath=False,
    ) or []

    authored = [
        curve
        for curve in curves
        if cmds.attributeQuery(
            "curvenetSegment",
            node=curve,
            exists=True,
        )
    ]

    if not authored:
        raise RuntimeError(
            f"No authored Curvenet segments found in {source_curve_group}."
        )

    return authored


def _world_matrix(node):
    values = cmds.xform(
        node,
        query=True,
        worldSpace=True,
        matrix=True,
    )
    return om.MMatrix(values)


def _source_region_triangles(source_mesh):
    """Encode the source preview's logical face ownership geometrically."""
    source_prefix = _short_name(source_mesh)
    preview_name = source_prefix + "CurvenetNode_curvenet_regionPreview"
    previews = cmds.ls(preview_name, long=True, type="transform") or []

    if not previews:
        raise RuntimeError(
            "The source Curvenet region preview is missing. "
            "Finish and connect the source Curvenet before transferring it."
        )

    preview = previews[0]
    region_attribute = preview + ".curvenetRegionByFace"

    preview_shape = _mesh_shape(preview)
    selection = om.MSelectionList()
    selection.add(preview_shape)
    dag_path = selection.getDagPath(0)
    mesh_fn = om.MFnMesh(dag_path)
    face_count = mesh_fn.numPolygons

    if cmds.objExists(region_attribute):
        encoded_region_ids = cmds.getAttr(region_attribute) or ""
        region_ids = [
            int(value)
            for value in encoded_region_ids.split(",")
            if value != ""
        ]
    else:
        # Existing scenes can contain a correct preview created before the
        # logical IDs were stored. Recover them from connected colour areas.
        face_colors = []
        all_face_vertex_colors = mesh_fn.getFaceVertexColors(
            mesh_fn.currentColorSetName(),
            om.MColor(),
        )

        for face_id in range(face_count):
            colors = [
                all_face_vertex_colors[
                    mesh_fn.getFaceVertexIndex(face_id, vertex_index)
                ]
                for vertex_index in range(
                    len(mesh_fn.getPolygonVertices(face_id))
                )
            ]

            if not colors:
                raise RuntimeError(
                    "The source Curvenet preview has no region colours."
                )

            inverse_count = 1.0 / len(colors)
            face_colors.append(
                tuple(
                    round(
                        sum(color[channel] for color in colors)
                        * inverse_count,
                        5,
                    )
                    for channel in range(3)
                )
            )

        region_ids = [-1] * face_count
        face_iterator = om.MItMeshPolygon(dag_path)
        region_id = 0

        for first_face_id in range(face_count):
            if region_ids[first_face_id] >= 0:
                continue

            region_ids[first_face_id] = region_id
            pending = [first_face_id]

            while pending:
                face_id = pending.pop()
                face_iterator.setIndex(face_id)

                for neighbour_id in face_iterator.getConnectedFaces():
                    if (
                        region_ids[neighbour_id] < 0
                        and face_colors[neighbour_id] == face_colors[face_id]
                    ):
                        region_ids[neighbour_id] = region_id
                        pending.append(neighbour_id)

            region_id += 1

        print("Recovered source logical regions:", region_id)

    if len(region_ids) != face_count:
        raise RuntimeError(
            "Source Curvenet region metadata does not match its preview mesh."
        )

    points = mesh_fn.getPoints(om.MSpace.kObject)
    face_iterator = om.MItMeshPolygon(dag_path)
    triangles = []

    while not face_iterator.isDone():
        face_id = face_iterator.index()

        if face_id >= len(region_ids):
            raise RuntimeError(
                "Source Curvenet region metadata does not match its preview mesh."
            )

        region_id = region_ids[face_id]
        vertex_ids = list(face_iterator.getVertices())

        if region_id >= 0 and len(vertex_ids) >= 3:
            first = points[vertex_ids[0]]

            for index in range(1, len(vertex_ids) - 1):
                second = points[vertex_ids[index]]
                third = points[vertex_ids[index + 1]]
                values = (
                    region_id,
                    first.x, first.y, first.z,
                    second.x, second.y, second.z,
                    third.x, third.y, third.z,
                )
                triangles.append(
                    ",".join(
                        str(value) if position == 0 else f"{value:.17g}"
                        for position, value in enumerate(values)
                    )
                )

        face_iterator.next()

    if not triangles:
        raise RuntimeError(
            "The source Curvenet preview contains no logical face samples."
        )

    return ";".join(triangles)


def _transfer_world_point(
    point,
    source_inverse_matrix,
    target_world_matrix,
):
    source_local = om.MPoint(point) * source_inverse_matrix
    target_world = source_local * target_world_matrix
    return [target_world.x, target_world.y, target_world.z]


def _joint_hierarchy(root):
    joints = []

    def visit(joint):
        joints.append(joint)

        for child in cmds.listRelatives(
            joint,
            children=True,
            type="joint",
            fullPath=True,
        ) or []:
            visit(child)

    visit(cmds.ls(root, long=True)[0])
    return joints


def transfer_joint_hierarchy_to_mesh(
    source_root_joint,
    target_mesh,
    source_mesh="tubeA",
    connect_pose=True,
):
    """Duplicate a source skeleton into a target mesh's local frame."""
    if not cmds.objExists(source_root_joint):
        raise RuntimeError(f"Source root joint does not exist: {source_root_joint}")

    if not cmds.objExists(source_mesh):
        raise RuntimeError(f"Source mesh does not exist: {source_mesh}")

    if not cmds.objExists(target_mesh):
        raise RuntimeError(f"Target mesh does not exist: {target_mesh}")

    target_prefix = _short_name(target_mesh)
    target_root_name = target_prefix + "_skeleton_root"
    target_group_name = target_prefix + "_transferredSkeleton_GRP"

    if cmds.objExists(target_root_name) or cmds.objExists(target_group_name):
        raise RuntimeError(
            f"Target skeleton already exists for: {target_prefix}"
        )

    source_joints = _joint_hierarchy(source_root_joint)
    duplicated_root = cmds.duplicate(
        source_root_joint,
        renameChildren=True,
        returnRootsOnly=True,
    )[0]
    duplicated_root = cmds.rename(duplicated_root, target_root_name)
    target_group = cmds.group(empty=True, name=target_group_name)
    duplicated_root = cmds.parent(
        duplicated_root,
        target_group,
        absolute=True,
    )[0]
    target_joints = _joint_hierarchy(duplicated_root)

    if len(source_joints) != len(target_joints):
        cmds.delete(duplicated_root)
        raise RuntimeError("Duplicated joint hierarchy does not match the source.")

    source_inverse_matrix = _world_matrix(source_mesh).inverse()
    target_world_matrix = _world_matrix(target_mesh)
    transfer_matrix = source_inverse_matrix * target_world_matrix
    cmds.xform(
        target_group,
        worldSpace=True,
        matrix=list(transfer_matrix),
    )
    duplicated_root = cmds.ls(duplicated_root, long=True)[0]
    target_joints = _joint_hierarchy(duplicated_root)

    if connect_pose:
        for source_joint, target_joint in zip(source_joints, target_joints):
            cmds.connectAttr(
                source_joint + ".rotate",
                target_joint + ".rotate",
                force=True,
            )
            cmds.connectAttr(
                source_joint + ".scale",
                target_joint + ".scale",
                force=True,
            )

    cmds.addAttr(
        target_group,
        longName="transferredCurvenetSkeleton",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(
        target_group + ".transferredCurvenetSkeleton",
        lock=True,
    )
    print("Transferred skeleton to:", target_mesh)
    print("Transferred joints:", len(target_joints))
    print("Target root:", duplicated_root)
    return duplicated_root, target_joints


def _project_world_point(point, target_mesh):
    target_shape = _mesh_shape(target_mesh)
    closest_point = cmds.createNode("closestPointOnMesh")

    try:
        cmds.connectAttr(
            target_shape + ".worldMesh[0]",
            closest_point + ".inMesh",
            force=True,
        )
        cmds.connectAttr(
            target_shape + ".worldMatrix[0]",
            closest_point + ".inputMatrix",
            force=True,
        )
        cmds.setAttr(
            closest_point + ".inPosition",
            point[0],
            point[1],
            point[2],
            type="double3",
        )
        return list(cmds.getAttr(closest_point + ".position")[0])
    finally:
        cmds.delete(closest_point)


def _deformer_input_mesh_plug(deformer):
    sources = cmds.listConnections(
        deformer + ".input[0].inputGeometry",
        source=True,
        destination=False,
        plugs=True,
    ) or []

    if not sources:
        raise RuntimeError(
            f"Could not find the upstream mesh connected to {deformer}."
        )

    source_node = sources[0].split(".", 1)[0]
    return source_node + ".worldMesh[0]"


def _create_target_groups(target_prefix):
    root_group = target_prefix + "_transferredCurvenet_GRP"

    if cmds.objExists(root_group):
        cmds.delete(root_group)

    root_group = cmds.group(empty=True, name=root_group)
    node_group = cmds.group(
        empty=True,
        name=target_prefix + "_transferredCurvenet_nodes_GRP",
        parent=root_group,
    )
    projected_group = cmds.group(
        empty=True,
        name=target_prefix + "_transferredCurvenet_projectedCurves_GRP",
        parent=root_group,
    )
    return root_group, node_group, projected_group


def _create_node_marker(
    target_prefix,
    node_id,
    position,
    node_group,
    radius,
):
    marker = cmds.polySphere(
        name=f"{target_prefix}_CN_node_{node_id}",
        radius=radius,
        subdivisionsX=12,
        subdivisionsY=12,
    )[0]
    cmds.xform(marker, worldSpace=True, translation=position)
    cmds.parent(marker, node_group)
    cmds.addAttr(
        marker,
        longName="transferredCurvenetNode",
        attributeType="bool",
        defaultValue=True,
    )
    cmds.setAttr(marker + ".transferredCurvenetNode", lock=True)
    cmds.addAttr(
        marker,
        longName="curvenetLogicalNodeId",
        attributeType="long",
    )
    cmds.setAttr(marker + ".curvenetLogicalNodeId", node_id)
    cmds.setAttr(marker + ".curvenetLogicalNodeId", lock=True)
    return marker


def _source_logical_node_id(control):
    attribute = control + ".curvenetLogicalNodeId"

    if cmds.objExists(attribute):
        return cmds.getAttr(attribute)

    match = re.search(r"(\d+)$", control.rsplit("|", 1)[-1])

    if not match:
        raise RuntimeError(f"No logical Curvenet node ID found for {control}.")

    return int(match.group(1))


def _create_endpoint_expression(curve, start_marker, end_marker):
    shape = cmds.listRelatives(
        curve,
        shapes=True,
        noIntermediate=True,
        fullPath=False,
    )[0]
    control_count = cmds.getAttr(shape + ".controlPoints", size=True)
    last_control = control_count - 1
    start_rest = cmds.getAttr(start_marker + ".translate")[0]
    end_rest = cmds.getAttr(end_marker + ".translate")[0]

    lines = []

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
                f"({start_marker}.translate{axis} - "
                f"{start_rest[component]:.17g}) + "
                f"{end_weight:.17g} * "
                f"({end_marker}.translate{axis} - "
                f"{end_rest[component]:.17g});"
            )

    cmds.expression(
        name=curve + "_endpointExpr",
        string="\n".join(lines),
        alwaysEvaluate=False,
        unitConversion="all",
    )


def attach_existing_curvenet_to_mesh(
    target_mesh,
    source_mesh="tubeA",
    source_curve_group="tubeA_drawnCurvenet_curves_GRP",
    full_surface=None,
):
    """Transfer one authored Curvenet to another mesh and bind it."""
    if not cmds.objExists(source_mesh):
        raise RuntimeError(f"Source mesh does not exist: {source_mesh}")

    if not cmds.objExists(target_mesh):
        raise RuntimeError(f"Target mesh does not exist: {target_mesh}")

    source_segments = _authored_segments(source_curve_group)
    source_region_triangles = _source_region_triangles(source_mesh)
    target_prefix = _short_name(target_mesh)
    root_group, node_group, projected_group = _create_target_groups(target_prefix)
    cmds.addAttr(
        root_group,
        longName="transferredRegionTriangles",
        dataType="string",
    )
    cmds.setAttr(
        root_group + ".transferredRegionTriangles",
        source_region_triangles,
        type="string",
    )

    if full_surface is None:
        source_prefix = _short_name(source_mesh)
        coverage_attribute = source_prefix + "CurvenetNode.fullSurfaceCurvenet"
        full_surface = (
            bool(cmds.getAttr(coverage_attribute))
            if cmds.objExists(coverage_attribute)
            else False
        )

    source_controls = []

    for source_curve in source_segments:
        for control in _endpoint_controls(source_curve):
            if control not in source_controls:
                source_controls.append(control)

    source_bounds = cmds.exactWorldBoundingBox(source_mesh)
    target_bounds = cmds.exactWorldBoundingBox(target_mesh)
    source_diagonal = sum(
        (source_bounds[index + 3] - source_bounds[index]) ** 2
        for index in range(3)
    ) ** 0.5
    target_diagonal = sum(
        (target_bounds[index + 3] - target_bounds[index]) ** 2
        for index in range(3)
    ) ** 0.5
    source_radii = []

    for control in source_controls:
        bounds = cmds.exactWorldBoundingBox(control)
        source_radii.append(
            0.5 * (
                (bounds[3] - bounds[0])
                + (bounds[4] - bounds[1])
                + (bounds[5] - bounds[2])
            ) / 3.0
        )

    source_radius = (
        sum(source_radii) / len(source_radii)
        if source_radii else max(source_diagonal * 0.006, 1.0e-5)
    )
    marker_radius = source_radius * (
        target_diagonal / source_diagonal
        if source_diagonal > 1.0e-12 else 1.0
    )

    source_inverse_matrix = _world_matrix(source_mesh).inverse()
    target_world_matrix = _world_matrix(target_mesh)
    projected_endpoint_by_control = {}
    marker_by_control = {}
    node_id_by_control = {}
    projected_curves = []
    surface_projector = _ContinuousSurfaceProjector(target_mesh)

    def projected_endpoint(control):
        if control not in projected_endpoint_by_control:
            source_position = cmds.xform(
                control,
                query=True,
                worldSpace=True,
                translation=True,
            )
            transferred = _transfer_world_point(
                source_position,
                source_inverse_matrix,
                target_world_matrix,
            )
            projected_endpoint_by_control[control] = _project_world_point(
                transferred,
                target_mesh,
            )
            logical_node_id = _source_logical_node_id(control)
            marker_by_control[control] = _create_node_marker(
                target_prefix,
                logical_node_id,
                projected_endpoint_by_control[control],
                node_group,
                marker_radius,
            )
            node_id_by_control[control] = logical_node_id

        return projected_endpoint_by_control[control]

    for curve_id, source_curve in enumerate(source_segments):
        start_control, end_control = _endpoint_controls(source_curve)
        start_point = projected_endpoint(start_control)
        end_point = projected_endpoint(end_control)
        transferred_points = [start_point]

        for sample_index in range(1, PROJECTED_SAMPLES - 1):
            parameter = sample_index / float(PROJECTED_SAMPLES - 1)
            source_point = _point_on_curve(source_curve, parameter)
            transferred = _transfer_world_point(
                source_point,
                source_inverse_matrix,
                target_world_matrix,
            )
            transferred_points.append(transferred)

        transferred_points.append(end_point)
        points = surface_projector.project_polyline(
            transferred_points,
            start_point,
            end_point,
        )

        projected_curve = cmds.curve(
            name=f"{target_prefix}_CN_projected_{curve_id}",
            degree=1,
            point=points,
        )
        cmds.parent(projected_curve, projected_group)
        cmds.addAttr(
            projected_curve,
            longName="projectedCurvenetProfile",
            attributeType="bool",
            defaultValue=True,
        )
        cmds.setAttr(
            projected_curve + ".projectedCurvenetProfile",
            lock=True,
        )
        for attribute, marker in (
            ("curvenetStartControl", marker_by_control[start_control]),
            ("curvenetEndControl", marker_by_control[end_control]),
        ):
            cmds.addAttr(
                projected_curve,
                longName=attribute,
                attributeType="message",
            )
            cmds.connectAttr(
                marker + ".message",
                projected_curve + "." + attribute,
                force=True,
            )
        _create_endpoint_expression(
            projected_curve,
            marker_by_control[start_control],
            marker_by_control[end_control],
        )
        projected_curves.append(projected_curve)

    deformer_name = target_prefix + "CurvenetNode"
    preview_group = deformer_name + "_curvenet_group"

    if cmds.objExists(deformer_name):
        cmds.delete(deformer_name)

    if cmds.objExists(preview_group):
        cmds.delete(preview_group)

    deformer = cmds.deformer(
        target_mesh,
        type="curvenetNode",
        name=deformer_name,
    )[0]
    cmds.setAttr(
        deformer + ".fullSurfaceCurvenet",
        bool(full_surface),
    )
    cmds.setAttr(
        deformer + ".transferredRegionTriangles",
        source_region_triangles,
        type="string",
    )
    cmds.connectAttr(
        _deformer_input_mesh_plug(deformer),
        deformer + ".inputMesh",
        force=True,
    )

    for curve_id, curve in enumerate(projected_curves):
        curve_shape = cmds.listRelatives(
            curve,
            shapes=True,
            noIntermediate=True,
            fullPath=True,
        )[0]
        cmds.connectAttr(
            curve_shape + ".worldSpace[0]",
            f"{deformer}.inputCurves[{curve_id}]",
            force=True,
        )
        source_curve = source_segments[curve_id]
        start_control, end_control = _endpoint_controls(source_curve)
        cmds.setAttr(
            f"{deformer}.inputCurveStartNodeIds[{curve_id}]",
            node_id_by_control[start_control],
        )
        cmds.setAttr(
            f"{deformer}.inputCurveEndNodeIds[{curve_id}]",
            node_id_by_control[end_control],
        )

    print("Transferred Curvenet to:", target_mesh)
    print("Projected curves:", len(projected_curves))
    print("Shared projected endpoint controls:", len(projected_endpoint_by_control))
    print("Deformer:", deformer)
    return deformer


def attach_existing_curvenet_to_selected_mesh():
    """Attach Tube A's authored Curvenet to the selected target mesh."""
    selection = cmds.ls(selection=True, long=True, type="transform") or []

    if len(selection) != 1:
        raise RuntimeError("Select exactly one target mesh transform.")

    target_mesh = selection[0]
    _mesh_shape(target_mesh)
    return attach_existing_curvenet_to_mesh(target_mesh)
