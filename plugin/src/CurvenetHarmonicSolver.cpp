#include "CurvenetHarmonicSolver.h"
#include "GeometryUtils.h"

#include <algorithm>
#include <unordered_set>

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

            const Point3 displacement =
                GeometryUtils::interpolateSegmentDisplacement(
                    neutral[constraint.segmentId],
                    neutral[constraint.segmentId + 1],
                    current[constraint.segmentId],
                    current[constraint.segmentId + 1],
                    constraint.segmentT
                );
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

    return std::vector<Point3>(
        displacements.begin(),
        displacements.begin() + outputVertexCount
    );
}

bool CurvenetHarmonicSolver::isInitialized() const
{
    return outputVertexCount > 0 &&
        !adjacentVertexIds.empty() &&
        adjacentVertexIds.size() == displacements.size();
}
