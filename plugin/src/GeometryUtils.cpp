#include "GeometryUtils.h"
#include <cmath>

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

    double GeometryUtils::length(
        const Point3& vector
    )
    {
        return std::sqrt(
            dot(vector, vector)
        );
    }

    double GeometryUtils::pointToPointDistance(
        const Point3& a,
        const Point3& b
    )
    {
        Point3 direction =
            subtract(a, b);

        return length(direction);
    }

    double GeometryUtils::clamp(
        double value,
        double minimum,
        double maximum
    )
    {
        if (value < minimum)
        {
            return minimum;
        }

        if (value > maximum)
        {
            return maximum;
        }

        return value;
    }

    Point3 GeometryUtils::addScaled(
        const Point3& start,
        const Point3& direction,
        double scale
    )
    {
        return Point3{
            start.x + direction.x * scale,
            start.y + direction.y * scale,
            start.z + direction.z * scale
        };
    }
}
