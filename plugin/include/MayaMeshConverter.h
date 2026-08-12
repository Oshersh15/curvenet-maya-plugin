#pragma once

#include "HalfEdge.h"

#include <maya/MFnMesh.h>

class MayaMeshConverter
{
public:
    static MStatus buildFromMayaMesh(
        const MFnMesh& meshFn,
        HalfEdgeMesh& mesh
    );
};
