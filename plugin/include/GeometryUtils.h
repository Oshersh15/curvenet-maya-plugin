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

    /*
        Computes the dot product between two 3D vectors.

        The dot product measures how much two vectors point in the same
        direction. It is used by closest-point and intersection algorithms.
    */
    double dot(
        const Point3& a,
        const Point3& b
    );

    /*
        Returns the Euclidean length (magnitude) of a 3D vector.
    */
    double length(
        const Point3& vector
    );

    /*
        Returns the Euclidean distance between two 3D points.
    */
    double pointToPointDistance(
        const Point3& a,
        const Point3& b
    );

    /*
        Restricts a value to the range [minimum, maximum].

        If the value is smaller than the minimum, the minimum is returned.
        If the value is larger than the maximum, the maximum is returned.
    */
    double clamp(
        double value,
        double minimum,
        double maximum
    );
}
