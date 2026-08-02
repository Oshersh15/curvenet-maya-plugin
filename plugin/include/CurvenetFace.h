#pragma once

#include <vector>

struct CurvenetFaceBoundary
{
    /*
        Profile curve containing this
        boundary section.
    */
    int curveId = -1;

    /*
        Mesh vertices marking the two shared
        Curvenet nodes at the ends of this section.
    */
    int startVertexId = -1;
    int endVertexId = -1;

    /*
        Whether this section is traversed opposite
        to the stored CutChain direction.
    */
    bool reversed = false;
};

struct CurvenetFace
{
    /*
        Identifier for this logical Curvenet face.
    */
    int id = -1;

    /*
        Ordered and oriented profile-curve traversal
        around the boundary of this face.
    */
    std::vector<CurvenetFaceBoundary> boundary;
};
