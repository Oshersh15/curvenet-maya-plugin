#pragma once

#include "HalfEdge.h"

/*
    Utility functions for simple 3D geometry operations.

    These functions are independent from the Maya API so they can be
    tested with GoogleTest and reused by curve/mesh intersection code.
*/
namespace GeometryUtils
{
    /*
        Returns the vector from b to a.

        Example:
        a = (5, 7, 2)
        b = (1, 4, 2)

        subtract(a, b) = (4, 3, 0)
    */
    Point3 subtract(
        const Point3& a,
        const Point3& b
    );
}
