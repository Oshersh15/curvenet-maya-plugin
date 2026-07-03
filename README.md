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

### Half-Edge Topology

- Vertex structure.
- Face structure.
- Half-edge structure.
- Directed edges.
- Face traversal.
- Adjacent vertex traversal.
- Twin edge detection.

### Maya Integration

- Custom Maya deformer node.
- Curve input attributes.
- Mesh input attributes.
- Automatic mesh conversion.
- HalfEdgeMesh generation from Maya meshes.

### Testing

- GoogleTest unit tests.
- Face traversal tests.
- Twin assignment tests.
- Invalid input tests.

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
- Curve connectivity
- Half-edge topology
- Maya mesh conversion
- Twin detection
- Unit testing

### In Progress

- Curve-mesh intersection detection
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
