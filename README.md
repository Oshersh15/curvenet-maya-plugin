# Curvenet Maya Plugin

A Maya C++ plugin inspired by Pixar's *Character Articulation through Profile Curves* (De Goes et al., 2022).

This project investigates curve-based representations for topology-independent articulation and deformation. The long-term goal is to explore whether a Curvenet representation can transfer articulation behaviour between different mesh topologies.

---

## Project Goals

- Investigate profile-curve based representations.
- Construct a Curvenet from profile curves.
- Develop a cut-mesh representation.
- Explore topology-independent articulation.
- Test deformation transfer between different meshes.

---

## Deformation Research

One of the central research questions of this project is how profile curves can drive mesh deformation independently of topology.

The original paper describes the concept of using profile curves for articulation but does not fully describe the underlying deformation implementation. As a result, this project investigates possible approaches for transferring articulation through a Curvenet representation.

Current areas of investigation include:

- Cut-mesh representations.
- Surface regions influenced by profile curves.
- Curve-driven deformation methods.
- Topology-independent articulation.
- Potential subdivision-based deformation approaches.

The current implementation focuses on constructing the Curvenet and its relationship to the mesh topology before implementing deformation behaviour.

---

## Current Features

### Curvenet Representation

- Multiple profile curves.
- Curve connectivity detection.
- Curve adjacency graph.
- Dense profile curve sampling.
- Uniform arc-length polyline generation.
- Adaptive curve sampling based on mesh resolution.
- Initial curve–mesh crossing detection.

### Half-Edge Topology

- Vertex structure.
- Face structure.
- Half-edge structure.
- Directed edges.
- Face traversal.
- Adjacent vertex traversal.
- Twin edge detection.

### Geometry Utilities

- Point subtraction
- Dot product
- Vector length
- Point-to-point distance
- Value clamping
- Point evaluation along a segment
- Closest point on a segment
- Segment-to-segment distance

### Maya Integration

- Custom Maya deformer node.
- Curve input attributes.
- Mesh input attributes.
- Automatic mesh conversion.
- HalfEdgeMesh generation from Maya meshes.
- Maya NURBS curve evaluation.
- Adaptive sampling using mesh edge resolution.

### Testing

- GoogleTest unit tests.
- Face traversal tests.
- Twin assignment tests.
- Profile curve sampling tests.
- Invalid input tests.

### Debug Visualisation

- Maya command for generating Curvenet debug geometry.
- Degree-1 visualisation of sampled profile-curve polylines.
- Locator markers for detected curve-mesh crossings.
- Automatic replacement of existing debug objects.
- Locator markers for detected endpoint-to-curve connections.
- Colour-coded profile curves for easier Curvenet inspection.

### Cut Path Representation

- CutPath structure representing one complete cut per profile curve.
- Stores ordered CutCrossing records.
- Supports multiple CutPaths for multiple profile curves.
- Stable crossing ordering using curve segment parameterisation.
- Derives traversed face intervals between consecutive crossings.
- Stores unique influenced mesh faces in traversal order.
- Collects the unique mesh vertices belonging to influenced faces.
- Stores the influenced vertex set for each CutPath.

### Vertex-to-Curve Binding

- Binds influenced mesh vertices to their closest sampled profile-curve segment.
- Stores the controlling segment ID and parameter along the segment.
- Stores the neutral vertex offset from the sampled curve.

### Curve-Driven Deformation

- Captures neutral and posed sampled profile-curve positions.
- Maintains a fixed neutral sample count during posing.
- Interpolates curve displacement using each vertex binding parameter.
- Applies deformation only to vertices influenced by the CutPath.

### Curvenet Connections

- Detects endpoint-to-curve connections between profile curves.
- Supports connections to any sampled location along a target curve.
- Stores target curve ID, sampled segment ID and segment parameter.
- Prevents duplicate connection records.
- Supports circular, vertical and diagonal profile-curve layouts.

### CutPath Face Splitting

Consecutive CutVertices can now be connected through their shared mesh faces.

After all crossed mesh edges have been split, the CutPath is processed in traversal order. Each consecutive pair of resulting mesh vertices is connected by a pair of directed twin half-edges. The existing face boundary is then divided into two valid face loops, preserving the original face ID for one region and creating a new face for the other.

Current-face lookup is performed against the modified topology so that later intervals do not rely on face IDs that may have become stale after earlier face splits.

The complete operation is integrated into `CutPathMeshSplitter::apply()`, extending CutPath processing from edge splitting to actual cut-edge construction through faces.

The face-splitting operation has been verified on several basic polygon configurations:

- a quad crossed through opposite edges, producing two quads;
- a quad crossed through adjacent edges, producing a triangle and a pentagon;
- a triangle crossed through two edges, producing a triangle and a quadrilateral.

These cases use the same general boundary-loop splitting algorithm rather than specialised logic for individual polygon or crossing types.

### Ordered Cut Chains

Face-level cut edges are assembled into ordered `CutChain` representations following the traversal order of the originating `CutPath`.

Each chain stores the resulting mesh vertex IDs and forward cut half-edge IDs in profile-curve order. Open and closed chains are represented explicitly. Closed chains include the final cut edge from the last vertex back to the first without duplicating the first vertex in the ordered vertex list.

This connects local face-splitting topology into complete profile-derived cut chains that can be used by later Curvenet topology and correspondence stages.

The ordered cut-chain representation has been verified on multiple profile configurations:

- vertical open chains;
- diagonal open chains crossing multiple faces and edge orientations;
- circular closed chains on Tube A.

The Tube A circular profile successfully produced a closed chain with matching vertex and cut-half-edge counts, and the final cut edge returned from the last chain vertex to the first.

Testing also exposed a separate near-coincident intersection case when a profile lies directly on a mesh edge loop. This is tracked as a robustness task rather than treated as normal chain behaviour.

### Multi-profile Curvenet Cutting

The cutting pipeline now supports complete Curvenets composed of multiple connected profile curves.

Implemented functionality includes:

- Processing multiple profile curves on a single evolving HalfEdge mesh.
- Progressive mesh modification so each profile operates on previously generated topology.
- Independent `CutChain` generation for every profile curve.
- Shared Curvenet node detection with embedded mesh vertex reuse.
- Support for both open and closed profile curves.
- Comprehensive unit and integration tests covering multi-profile cutting and shared-node behaviour.

The resulting embedded Curvenet provides the topological foundation required for the subsequent correspondence and deformation stages.

### Embedded Curvenet Representation

The complete mesh-specific Curvenet embedding is stored after multi-profile cutting.

The representation includes:

- mesh vertex IDs belonging to the embedded Curvenet;
- forward cut half-edge IDs for all profile-derived CutChains;
- mesh faces adjacent to the generated cut topology;
- direct profile-curve ID to CutChain mapping;
- shared Curvenet nodes mapped to their embedded mesh vertices;
- the profile curves connected through each shared node.

This separates the logical Curvenet structure from the original mesh topology while retaining the mesh-specific embedding required for later correspondence and deformation.

### Curvenet Face Representation

Logical Curvenet regions are reconstructed from the embedded profile-curve network.

Implemented functionality includes:

- Identification of closed regions bounded by connected profile curves.
- Ordered boundary traversal for every Curvenet face.
- Boundary orientation relative to the original CutChain direction.
- Support for Curvenet faces with arbitrary numbers of boundary sections.
- Unit tests covering both four-sided and five-sided Curvenet faces.

---

## Technologies

- C++17
- Autodesk Maya API
- CMake
- GoogleTest

---

## Building

```bash
mkdir build
cd build

cmake ..
cmake --build .
```

---

## Running Tests

```bash
cd build

ctest --output-on-failure
```

---

## Current Development Status

### Implemented

- Curvenet representation
- Uniform arc-length curve sampling
- Adaptive sampling density
- Curve connectivity
- Half-edge topology
- Maya mesh conversion
- Geometry utility library
- Closest point on a segment
- Segment-to-segment distance
- Initial curve-mesh crossing detection
- Twin detection
- Unit testing
- First curve-mesh crossing detection
- CutCrossing data representation
- First crossing record storage
- Multiple curve-mesh crossing detection
- Duplicate crossing filtering
- Ordered CutCrossing storage
- Curvenet debug visualiser
- Sampled polyline viewport display
- Curve-mesh crossing markers
- CutPath representation
- Ordered CutCrossing storage
- Multiple CutPath support
- Vertex-to-curve binding
- Neutral curve-offset storage

### In Progress

- Crossing record generation
- Cut-point generation
- Cut-path construction

### Planned

- Cut-mesh representation
- Curvenet articulation
- Mesh binding
- Topology-independent deformation

---

## Reference

De Goes, F., Sheffler, W., & Fleischer, K. (2022).

*Character Articulation through Profile Curves.*
