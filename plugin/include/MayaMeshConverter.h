#pragma once

/* Declares conversion from Maya mesh data to the internal half-edge mesh. */

#include "HalfEdge.h"

#include <maya/MFnMesh.h>

class MayaMeshConverter
{
public:
    static HalfEdgeMesh buildFromMayaMesh(const MFnMesh& meshFn);
};
