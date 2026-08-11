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

Point3 applyRotation(
    const std::array<double, 9>& rotation,
    const Point3& vector
)
{
    return Point3{
        rotation[0] * vector.x + rotation[1] * vector.y
            + rotation[2] * vector.z,
        rotation[3] * vector.x + rotation[4] * vector.y
            + rotation[5] * vector.z,
        rotation[6] * vector.x + rotation[7] * vector.y
            + rotation[8] * vector.z
    };
}

std::array<double, 9> fitRotationFromEdges(
    const std::vector<Point3>& neutralEdges,
    const std::vector<Point3>& currentEdges
)
{
    std::array<double, 9> identity = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    const int edgeCount = std::min(
        neutralEdges.size(),
        currentEdges.size()
    );

    if (edgeCount == 1)
    {
        const Point3& source = neutralEdges[0];
        const Point3& target = currentEdges[0];
        const double sourceLength = std::sqrt(
            source.x * source.x + source.y * source.y + source.z * source.z
        );
        const double targetLength = std::sqrt(
            target.x * target.x + target.y * target.y + target.z * target.z
        );

        if (sourceLength <= 1.0e-12 || targetLength <= 1.0e-12)
        {
            return identity;
        }

        const Point3 a{
            source.x / sourceLength,
            source.y / sourceLength,
            source.z / sourceLength
        };
        const Point3 b{
            target.x / targetLength,
            target.y / targetLength,
            target.z / targetLength
        };
        const Point3 axis{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
        const double cosine = std::max(
            -1.0,
            std::min(1.0, a.x * b.x + a.y * b.y + a.z * b.z)
        );
        const double sineSquared =
            axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;

        if (sineSquared <= 1.0e-12)
        {
            return identity;
        }

        const double factor = (1.0 - cosine) / sineSquared;
        return {
            cosine + axis.x * axis.x * factor,
            axis.x * axis.y * factor - axis.z,
            axis.x * axis.z * factor + axis.y,
            axis.y * axis.x * factor + axis.z,
            cosine + axis.y * axis.y * factor,
            axis.y * axis.z * factor - axis.x,
            axis.z * axis.x * factor - axis.y,
            axis.z * axis.y * factor + axis.x,
            cosine + axis.z * axis.z * factor
        };
    }

    if (edgeCount < 1)
    {
        return identity;
    }

    std::array<double, 9> covariance{};

    for (int edgeId = 0; edgeId < edgeCount; ++edgeId)
    {
        const double neutral[3] = {
            neutralEdges[edgeId].x,
            neutralEdges[edgeId].y,
            neutralEdges[edgeId].z
        };
        const double current[3] = {
            currentEdges[edgeId].x,
            currentEdges[edgeId].y,
            currentEdges[edgeId].z
        };

        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                covariance[row * 3 + column] +=
                    neutral[row] * current[column];
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

    for (int iteration = 0; iteration < 30; ++iteration)
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
            return identity;
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
    return {
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
        2.0 * (x * z + y * w), 2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y)
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

bool isApproximatelyRigid(
    const RigidTransform& transform,
    const std::vector<std::vector<Point3>>& neutralCurves,
    const std::vector<std::vector<Point3>>& currentCurves
)
{
    double squaredError = 0.0;
    double squaredScale = 0.0;
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
            const Point3 transformed = applyTransform(
                transform,
                neutralCurves[curveId][pointId]
            );
            const Point3& current = currentCurves[curveId][pointId];
            const double dx = transformed.x - current.x;
            const double dy = transformed.y - current.y;
            const double dz = transformed.z - current.z;
            squaredError += dx * dx + dy * dy + dz * dz;
            squaredScale +=
                neutralCurves[curveId][pointId].x *
                    neutralCurves[curveId][pointId].x +
                neutralCurves[curveId][pointId].y *
                    neutralCurves[curveId][pointId].y +
                neutralCurves[curveId][pointId].z *
                    neutralCurves[curveId][pointId].z;
            ++pointCount;
        }
    }

    if (pointCount == 0)
    {
        return false;
    }

    const double rootMeanSquareError = std::sqrt(
        squaredError / static_cast<double>(pointCount)
    );
    const double rootMeanSquareScale = std::sqrt(
        squaredScale / static_cast<double>(pointCount)
    );

    return rootMeanSquareError <=
        std::max(1.0e-7, rootMeanSquareScale * 0.0025);
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
    RigidTransform rigidTransform = fitRigidTransform(
        neutralCurves,
        currentSampledCurves
    );

    if (!isApproximatelyRigid(
            rigidTransform,
            neutralCurves,
            currentSampledCurves
        ))
    {
        rigidTransform = RigidTransform{};
    }

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

    std::vector<Point3> positions(displacements.size(), Point3{});

    for (int vertexId = 0;
         vertexId < static_cast<int>(positions.size());
         ++vertexId)
    {
        const Point3 rigidPosition = applyTransform(
            rigidTransform,
            neutralVertexPositions[vertexId]
        );
        positions[vertexId] = Point3{
            rigidPosition.x + displacements[vertexId].x,
            rigidPosition.y + displacements[vertexId].y,
            rigidPosition.z + displacements[vertexId].z
        };
    }

    /* A positional Laplacian follows translating constraints, but it blends
       through the inside of a rotation and visibly loses volume. Derive local
       rotations from the posed curve tangents, diffuse those rotations over
       the cut mesh, and reconstruct neutral edge vectors in those frames. */
    const std::array<double, 9> identity = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    std::vector<std::array<double, 9>> localRotations(
        positions.size(),
        identity
    );
    std::vector<std::array<double, 9>> nextRotations = localRotations;
    std::vector<bool> hasCurveRotation(positions.size(), false);
    std::vector<Point3> refined = positions;

    for (int vertexId = 0;
         vertexId < static_cast<int>(constraintsByVertex.size());
         ++vertexId)
    {
        std::vector<Point3> neutralTangents;
        std::vector<Point3> currentTangents;

        for (const CurveConstraint& constraint : constraintsByVertex[vertexId])
        {
            if (constraint.curveId < 0 ||
                constraint.curveId >= static_cast<int>(neutralCurves.size()) ||
                constraint.curveId >=
                    static_cast<int>(currentSampledCurves.size()))
            {
                continue;
            }

            const std::vector<Point3>& neutral =
                neutralCurves[constraint.curveId];
            const std::vector<Point3>& current =
                currentSampledCurves[constraint.curveId];

            if (constraint.segmentId < 0 ||
                constraint.segmentId + 1 >= static_cast<int>(neutral.size()) ||
                constraint.segmentId + 1 >= static_cast<int>(current.size()))
            {
                continue;
            }

            const Point3 neutralTangent{
                neutral[constraint.segmentId + 1].x
                    - neutral[constraint.segmentId].x,
                neutral[constraint.segmentId + 1].y
                    - neutral[constraint.segmentId].y,
                neutral[constraint.segmentId + 1].z
                    - neutral[constraint.segmentId].z
            };
            neutralTangents.push_back(neutralTangent);
            currentTangents.push_back(Point3{
                current[constraint.segmentId + 1].x
                    - current[constraint.segmentId].x,
                current[constraint.segmentId + 1].y
                    - current[constraint.segmentId].y,
                current[constraint.segmentId + 1].z
                    - current[constraint.segmentId].z
            });
        }

        if (!neutralTangents.empty())
        {
            localRotations[vertexId] = fitRotationFromEdges(
                neutralTangents,
                currentTangents
            );
            hasCurveRotation[vertexId] = true;
        }
    }

    const std::vector<Point3> basis = {
        Point3{1.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{0.0, 0.0, 1.0}
    };

    for (int iteration = 0; iteration < std::max(1, iterationCount); ++iteration)
    {
        for (int vertexId = 0;
             vertexId < static_cast<int>(localRotations.size());
             ++vertexId)
        {
            if (hasCurveRotation[vertexId] ||
                adjacentVertexIds[vertexId].empty())
            {
                nextRotations[vertexId] = localRotations[vertexId];
                continue;
            }

            std::vector<Point3> averageBasis(3, Point3{});

            for (int neighbourId : adjacentVertexIds[vertexId])
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    const Point3 transformed = applyRotation(
                        localRotations[neighbourId],
                        basis[axis]
                    );
                    averageBasis[axis].x += transformed.x;
                    averageBasis[axis].y += transformed.y;
                    averageBasis[axis].z += transformed.z;
                }
            }

            nextRotations[vertexId] = fitRotationFromEdges(
                basis,
                averageBasis
            );
        }

        localRotations.swap(nextRotations);
    }

    for (int relaxationIteration = 0;
         relaxationIteration < std::max(1, iterationCount);
         ++relaxationIteration)
    {
        for (int vertexId = 0;
             vertexId < static_cast<int>(positions.size());
             ++vertexId)
        {
            if (prescribedCounts[vertexId] > 0 ||
                adjacentVertexIds[vertexId].empty())
            {
                const Point3 rigidPosition = applyTransform(
                    rigidTransform,
                    neutralVertexPositions[vertexId]
                );
                refined[vertexId] = Point3{
                    rigidPosition.x + prescribed[vertexId].x,
                    rigidPosition.y + prescribed[vertexId].y,
                    rigidPosition.z + prescribed[vertexId].z
                };
                continue;
            }

            Point3 average;

            for (int neighbourId : adjacentVertexIds[vertexId])
            {
                const Point3 neutralEdge{
                    neutralVertexPositions[vertexId].x
                        - neutralVertexPositions[neighbourId].x,
                    neutralVertexPositions[vertexId].y
                        - neutralVertexPositions[neighbourId].y,
                    neutralVertexPositions[vertexId].z
                        - neutralVertexPositions[neighbourId].z
                };
                const Point3 rotatedHere = applyRotation(
                    localRotations[vertexId],
                    neutralEdge
                );
                const Point3 rotatedThere = applyRotation(
                    localRotations[neighbourId],
                    neutralEdge
                );
                average.x += positions[neighbourId].x
                    + 0.5 * (rotatedHere.x + rotatedThere.x);
                average.y += positions[neighbourId].y
                    + 0.5 * (rotatedHere.y + rotatedThere.y);
                average.z += positions[neighbourId].z
                    + 0.5 * (rotatedHere.z + rotatedThere.z);
            }

            const double inverseCount = 1.0 /
                static_cast<double>(adjacentVertexIds[vertexId].size());
            refined[vertexId] = Point3{
                average.x * inverseCount,
                average.y * inverseCount,
                average.z * inverseCount
            };
        }

        positions.swap(refined);
    }

    std::vector<Point3> result(outputVertexCount, Point3{});

    for (int vertexId = 0; vertexId < outputVertexCount; ++vertexId)
    {
        result[vertexId] = Point3{
            positions[vertexId].x - neutralVertexPositions[vertexId].x,
            positions[vertexId].y - neutralVertexPositions[vertexId].y,
            positions[vertexId].z - neutralVertexPositions[vertexId].z
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
