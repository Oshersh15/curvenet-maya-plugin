#include "CurvenetHarmonicSolver.h"
#include "GeometryUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_set>

namespace
{
struct RigidTransform
{
    std::array<double, 9> rotation = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    Point3 translation;
};

Point3 applyTransform(const RigidTransform& transform, const Point3& point)
{
    return Point3{
        transform.rotation[0] * point.x
            + transform.rotation[1] * point.y
            + transform.rotation[2] * point.z
            + transform.translation.x,
        transform.rotation[3] * point.x
            + transform.rotation[4] * point.y
            + transform.rotation[5] * point.z
            + transform.translation.y,
        transform.rotation[6] * point.x
            + transform.rotation[7] * point.y
            + transform.rotation[8] * point.z
            + transform.translation.z
    };
}

Point3 interpolatePoint(const Point3& start, const Point3& end, double t)
{
    return Point3{
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
        start.z + (end.z - start.z) * t
    };
}

RigidTransform fitRigidTransform(
    const std::vector<std::vector<Point3>>& neutralCurves,
    const std::vector<std::vector<Point3>>& currentCurves
)
{
    Point3 neutralCentre;
    Point3 currentCentre;
    int pointCount = 0;

    for (int curveId = 0;
         curveId < static_cast<int>(neutralCurves.size()) &&
             curveId < static_cast<int>(currentCurves.size());
         ++curveId)
    {
        const int count = std::min(
            neutralCurves[curveId].size(),
            currentCurves[curveId].size()
        );

        for (int pointId = 0; pointId < count; ++pointId)
        {
            neutralCentre.x += neutralCurves[curveId][pointId].x;
            neutralCentre.y += neutralCurves[curveId][pointId].y;
            neutralCentre.z += neutralCurves[curveId][pointId].z;
            currentCentre.x += currentCurves[curveId][pointId].x;
            currentCentre.y += currentCurves[curveId][pointId].y;
            currentCentre.z += currentCurves[curveId][pointId].z;
            ++pointCount;
        }
    }

    RigidTransform transform;

    if (pointCount < 3)
    {
        return transform;
    }

    const double inverseCount = 1.0 / static_cast<double>(pointCount);
    neutralCentre.x *= inverseCount;
    neutralCentre.y *= inverseCount;
    neutralCentre.z *= inverseCount;
    currentCentre.x *= inverseCount;
    currentCentre.y *= inverseCount;
    currentCentre.z *= inverseCount;
    std::array<double, 9> covariance{};

    for (int curveId = 0;
         curveId < static_cast<int>(neutralCurves.size()) &&
             curveId < static_cast<int>(currentCurves.size());
         ++curveId)
    {
        const int count = std::min(
            neutralCurves[curveId].size(),
            currentCurves[curveId].size()
        );

        for (int pointId = 0; pointId < count; ++pointId)
        {
            const Point3 neutral{
                neutralCurves[curveId][pointId].x - neutralCentre.x,
                neutralCurves[curveId][pointId].y - neutralCentre.y,
                neutralCurves[curveId][pointId].z - neutralCentre.z
            };
            const Point3 current{
                currentCurves[curveId][pointId].x - currentCentre.x,
                currentCurves[curveId][pointId].y - currentCentre.y,
                currentCurves[curveId][pointId].z - currentCentre.z
            };
            const double neutralValues[3] = {
                neutral.x, neutral.y, neutral.z
            };
            const double currentValues[3] = {
                current.x, current.y, current.z
            };

            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 3; ++column)
                {
                    covariance[row * 3 + column] +=
                        neutralValues[row] * currentValues[column];
                }
            }
        }
    }

    const double sxx = covariance[0];
    const double sxy = covariance[1];
    const double sxz = covariance[2];
    const double syx = covariance[3];
    const double syy = covariance[4];
    const double syz = covariance[5];
    const double szx = covariance[6];
    const double szy = covariance[7];
    const double szz = covariance[8];
    const double trace = sxx + syy + szz;
    const std::array<double, 16> horn = {
        trace, syz - szy, szx - sxz, sxy - syx,
        syz - szy, sxx - syy - szz, sxy + syx, szx + sxz,
        szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy,
        sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz
    };
    double eigenvalueShift = 0.0;

    for (double value : horn)
    {
        eigenvalueShift += std::abs(value);
    }

    std::array<double, 4> quaternion = {1.0, 0.0, 0.0, 0.0};

    for (int iteration = 0; iteration < 60; ++iteration)
    {
        std::array<double, 4> next{};

        for (int row = 0; row < 4; ++row)
        {
            for (int column = 0; column < 4; ++column)
            {
                next[row] += horn[row * 4 + column] * quaternion[column];

                if (row == column)
                {
                    next[row] += eigenvalueShift * quaternion[column];
                }
            }
        }

        const double length = std::sqrt(
            next[0] * next[0] + next[1] * next[1]
            + next[2] * next[2] + next[3] * next[3]
        );

        if (length <= 1.0e-12)
        {
            break;
        }

        for (int component = 0; component < 4; ++component)
        {
            quaternion[component] = next[component] / length;
        }
    }

    const double w = quaternion[0];
    const double x = quaternion[1];
    const double y = quaternion[2];
    const double z = quaternion[3];
    transform.rotation = {
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
        2.0 * (x * z + y * w), 2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y)
    };
    const Point3 rotatedCentre = applyTransform(transform, neutralCentre);
    transform.translation = Point3{
        currentCentre.x - rotatedCentre.x,
        currentCentre.y - rotatedCentre.y,
        currentCentre.z - rotatedCentre.z
    };
    return transform;
}
}

void CurvenetHarmonicSolver::initialize(
    const HalfEdgeMesh& cutMesh,
    const std::unordered_map<int, CutChain>& cutChainsByCurveId,
    int originalVertexCount,
    const std::vector<std::vector<Point3>>& neutralSampledCurves
)
{
    outputVertexCount = std::max(
        0,
        std::min(
            originalVertexCount,
            static_cast<int>(cutMesh.vertices.size())
        )
    );
    neutralCurves = neutralSampledCurves;
    neutralVertexPositions.clear();
    neutralVertexPositions.reserve(cutMesh.vertices.size());

    for (const Vertex& vertex : cutMesh.vertices)
    {
        neutralVertexPositions.push_back(vertex.position);
    }
    adjacentVertexIds.assign(cutMesh.vertices.size(), {});
    constraintsByVertex.assign(cutMesh.vertices.size(), {});
    displacements.assign(cutMesh.vertices.size(), {});

    std::vector<std::unordered_set<int>> uniqueNeighbours(
        cutMesh.vertices.size()
    );

    for (const HalfEdge& halfEdge : cutMesh.halfEdges)
    {
        if (halfEdge.startVertex < 0 ||
            halfEdge.endVertex < 0 ||
            halfEdge.startVertex >= static_cast<int>(cutMesh.vertices.size()) ||
            halfEdge.endVertex >= static_cast<int>(cutMesh.vertices.size()) ||
            halfEdge.startVertex == halfEdge.endVertex)
        {
            continue;
        }

        uniqueNeighbours[halfEdge.startVertex].insert(halfEdge.endVertex);
        uniqueNeighbours[halfEdge.endVertex].insert(halfEdge.startVertex);
    }

    for (int vertexId = 0;
         vertexId < static_cast<int>(uniqueNeighbours.size());
         ++vertexId)
    {
        adjacentVertexIds[vertexId].assign(
            uniqueNeighbours[vertexId].begin(),
            uniqueNeighbours[vertexId].end()
        );
        std::sort(
            adjacentVertexIds[vertexId].begin(),
            adjacentVertexIds[vertexId].end()
        );
    }

    for (const auto& entry : cutChainsByCurveId)
    {
        const int curveId = entry.first;

        for (const EmbeddedCurvePoint& point : entry.second.points)
        {
            if (point.meshVertexId < 0 ||
                point.meshVertexId >= static_cast<int>(constraintsByVertex.size()) ||
                point.curveSegmentId < 0)
            {
                continue;
            }

            constraintsByVertex[point.meshVertexId].push_back(
                CurveConstraint{
                    curveId,
                    point.curveSegmentId,
                    point.curveSegmentT
                }
            );
        }
    }
}

std::vector<Point3> CurvenetHarmonicSolver::solve(
    const std::vector<std::vector<Point3>>& currentSampledCurves,
    int iterationCount
)
{
    if (!isInitialized())
    {
        return {};
    }

    /* Start from the neutral displacement on every evaluation. Reusing the
       previous iterative state leaves visible residual deformation after the
       Curvenet returns to its neutral pose. */
    std::fill(
        displacements.begin(),
        displacements.end(),
        Point3{}
    );

    std::vector<Point3> prescribed(
        displacements.size(),
        Point3{}
    );
    std::vector<int> prescribedCounts(displacements.size(), 0);
    const RigidTransform rigidTransform = fitRigidTransform(
        neutralCurves,
        currentSampledCurves
    );

    for (int vertexId = 0;
         vertexId < static_cast<int>(constraintsByVertex.size());
         ++vertexId)
    {
        for (const CurveConstraint& constraint : constraintsByVertex[vertexId])
        {
            if (constraint.curveId < 0 ||
                constraint.curveId >= static_cast<int>(neutralCurves.size()) ||
                constraint.curveId >= static_cast<int>(currentSampledCurves.size()))
            {
                continue;
            }

            const std::vector<Point3>& neutral = neutralCurves[constraint.curveId];
            const std::vector<Point3>& current =
                currentSampledCurves[constraint.curveId];

            if (constraint.segmentId + 1 >= static_cast<int>(neutral.size()) ||
                constraint.segmentId + 1 >= static_cast<int>(current.size()))
            {
                continue;
            }

            const Point3 neutralPoint = interpolatePoint(
                neutral[constraint.segmentId],
                neutral[constraint.segmentId + 1],
                constraint.segmentT
            );
            const Point3 currentPoint = interpolatePoint(
                current[constraint.segmentId],
                current[constraint.segmentId + 1],
                constraint.segmentT
            );
            const Point3 rigidPoint = applyTransform(
                rigidTransform,
                neutralPoint
            );
            const Point3 displacement{
                currentPoint.x - rigidPoint.x,
                currentPoint.y - rigidPoint.y,
                currentPoint.z - rigidPoint.z
            };
            prescribed[vertexId].x += displacement.x;
            prescribed[vertexId].y += displacement.y;
            prescribed[vertexId].z += displacement.z;
            ++prescribedCounts[vertexId];
        }

        if (prescribedCounts[vertexId] > 0)
        {
            const double inverseCount =
                1.0 / static_cast<double>(prescribedCounts[vertexId]);
            prescribed[vertexId].x *= inverseCount;
            prescribed[vertexId].y *= inverseCount;
            prescribed[vertexId].z *= inverseCount;
            displacements[vertexId] = prescribed[vertexId];
        }
    }

    std::vector<Point3> next = displacements;

    for (int iteration = 0; iteration < std::max(1, iterationCount); ++iteration)
    {
        for (int vertexId = 0;
             vertexId < static_cast<int>(displacements.size());
             ++vertexId)
        {
            if (prescribedCounts[vertexId] > 0 ||
                adjacentVertexIds[vertexId].empty())
            {
                next[vertexId] = displacements[vertexId];
                continue;
            }

            Point3 average;

            for (int neighbourId : adjacentVertexIds[vertexId])
            {
                average.x += displacements[neighbourId].x;
                average.y += displacements[neighbourId].y;
                average.z += displacements[neighbourId].z;
            }

            const double inverseCount =
                1.0 / static_cast<double>(adjacentVertexIds[vertexId].size());
            average.x *= inverseCount;
            average.y *= inverseCount;
            average.z *= inverseCount;
            next[vertexId] = average;
        }

        displacements.swap(next);
    }

    std::vector<Point3> result(outputVertexCount, Point3{});

    for (int vertexId = 0; vertexId < outputVertexCount; ++vertexId)
    {
        const Point3 rigidPosition = applyTransform(
            rigidTransform,
            neutralVertexPositions[vertexId]
        );
        result[vertexId] = Point3{
            rigidPosition.x - neutralVertexPositions[vertexId].x
                + displacements[vertexId].x,
            rigidPosition.y - neutralVertexPositions[vertexId].y
                + displacements[vertexId].y,
            rigidPosition.z - neutralVertexPositions[vertexId].z
                + displacements[vertexId].z
        };
    }

    return result;
}

bool CurvenetHarmonicSolver::isInitialized() const
{
    return outputVertexCount > 0 &&
        !adjacentVertexIds.empty() &&
        adjacentVertexIds.size() == displacements.size();
}
