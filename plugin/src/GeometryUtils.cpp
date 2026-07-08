#include "GeometryUtils.h"

namespace GeometryUtils
{
    Point3 subtract(
        const Point3& a,
        const Point3& b
    )
    {
        return Point3{
            a.x - b.x,
            a.y - b.y,
            a.z - b.z
        };
    }

    double dot(
        const Point3& a,
        const Point3& b
    )
    {
        return
            a.x * b.x +
            a.y * b.y +
            a.z * b.z;
    }
}
