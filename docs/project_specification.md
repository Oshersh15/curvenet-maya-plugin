# Project Specification

# Topology-Independent Deformation Using Curvenets

## Project Goal

Develop a Maya C++ plugin inspired by Pixar's *Character Articulation through Profile Curves*. The project investigates Curvenets as a topology-independent rigging representation, where profile curves drive surface deformation and can be reused across different mesh topologies.

## Motivation

Traditional character deformation systems are often tightly coupled to mesh topology through skin weights and corrective shapes. This can make rigs difficult to transfer between meshes and expensive to maintain.

Pixar's *Character Articulation through Profile Curves* proposes Curvenets, a rigging representation based on profile curves that is independent of mesh topology.

## Research Question

How can the core concepts of Pixar's Curvenet-based deformation system be implemented within Maya, and to what extent can topology-independent deformation be achieved?

## Core Features

- Curvenet Creation
- Mesh Binding
- Surface Deformation
- Topology Independence

## Must Have Deliverables

- Working Maya C++ plugin
- Curvenet creation workflow
- Mesh binding workflow
- Surface deformation driven by Curvenets
- Demonstration using multiple mesh topologies
- Technical evaluation and documentation

## Nice To Have Features

- Curvenet skinning workflow
- User interface
- Driven key support
- Better deformation quality
- Performance improvements

## Evaluation

The system will be evaluated through:

- Deformation quality
- Usability
- Behaviour across different mesh topologies
- Comparison against simpler approaches developed during the proof of concept

## Target Demonstration

```text
Hand Mesh A
↓
Curvenet
↓
Pose Fingers
↓
Hand Deforms

Swap to Hand Mesh B
↓
Same Curvenet
↓
Same Finger Pose
↓
Hand Deforms
```
