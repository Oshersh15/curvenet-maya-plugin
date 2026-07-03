#pragma once

#include "HalfEdge.h"

#include <maya/MFnMesh.h>

class MayaMeshConverter
{
public:
    static HalfEdgeMesh buildFromMayaMesh(const MFnMesh& meshFn);
};
