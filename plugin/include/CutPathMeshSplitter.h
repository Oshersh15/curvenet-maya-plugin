#pragma once

#include "CutPath.h"
#include "HalfEdge.h"

#include <vector>

/*
    Stores the result of applying the ordered CutVertices
    from one CutPath to a HalfEdgeMesh.

    meshVertexIds preserves CutPath order:
    each entry identifies the mesh vertex created or reused
    for the corresponding CutVertex.
*/
struct CutPathSplitResult
{
    bool success = false;

    std::vector<int> meshVertexIds;
};

/*
    Applies the ordered CutVertices of a CutPath to a
    HalfEdgeMesh.

    The splitter is responsible for coordinating repeated
    edge splits. HalfEdgeMesh remains responsible for the
    individual boundary-edge and internal-edge operations.
*/
class CutPathMeshSplitter
{
public:
    static CutPathSplitResult apply(
        HalfEdgeMesh& mesh,
        const CutPath& cutPath,
        double duplicateTolerance
    );
};
