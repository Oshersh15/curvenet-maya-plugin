#include "CurvenetFaceRegionBuilder.h"
#include "GeometryUtils.h"

#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace
{
    struct BoundaryCollection
    {
        std::unordered_set<int>
            boundaryHalfEdgeIds;

        /*
            One oriented boundary half-edge belonging
            to this Curvenet face. Used only to choose
            the flood-fill seed.
        */
        int seedHalfEdgeId = -1;

        bool containsClosedCutChain = false;
    };

    bool collectBoundaryHalfEdges(
        const CurvenetFace& curvenetFace,
        const CurvenetCutResult& cutResult,
        BoundaryCollection& boundaryCollection
    )
{
    for (const CurvenetFaceBoundary& boundary :
         curvenetFace.boundary)
    {
        const auto chainIterator =
            cutResult.cutChainsByCurveId.find(
                boundary.curveId
            );

        if (chainIterator ==
            cutResult.cutChainsByCurveId.end())
        {
            return false;
        }

        const CutChain& cutChain =
            chainIterator->second;

        boundaryCollection.containsClosedCutChain =
            boundaryCollection.containsClosedCutChain ||
            cutChain.closed;

        if (cutChain.vertexIds.size() < 2 ||
            cutChain.halfEdgeIds.empty())
        {
            return false;
        }

        int startIndex = -1;
        int endIndex = -1;

        for (int vertexIndex = 0;
             vertexIndex <
                 static_cast<int>(
                     cutChain.vertexIds.size()
                 );
             ++vertexIndex)
        {
            if (cutChain.vertexIds[vertexIndex] ==
                boundary.startVertexId)
            {
                startIndex = vertexIndex;
            }

            if (cutChain.vertexIds[vertexIndex] ==
                boundary.endVertexId)
            {
                endIndex = vertexIndex;
            }
        }

        if (startIndex < 0 ||
            endIndex < 0 ||
            startIndex == endIndex)
        {
            return false;
        }

        const int vertexCount =
            static_cast<int>(
                cutChain.vertexIds.size()
            );

        if (!boundary.reversed)
        {
            int currentIndex =
                startIndex;

            while (currentIndex != endIndex)
            {
                if (currentIndex < 0 ||
                    currentIndex >=
                        static_cast<int>(
                            cutChain.halfEdgeIds.size()
                        ))
                {
                    return false;
                }

                const int orientedHalfEdgeId =
                    cutChain.halfEdgeIds[
                        currentIndex
                    ];

                boundaryCollection
                    .boundaryHalfEdgeIds
                    .insert(
                        orientedHalfEdgeId
                    );

                if (boundaryCollection.seedHalfEdgeId < 0)
                {
                    boundaryCollection.seedHalfEdgeId =
                        orientedHalfEdgeId;
                }

                ++currentIndex;

                if (currentIndex >= vertexCount)
                {
                    if (!cutChain.closed)
                    {
                        return false;
                    }

                    currentIndex = 0;
                }
            }
        }
        else
        {
            int currentIndex =
                startIndex;

            while (currentIndex != endIndex)
            {
                int previousIndex =
                    currentIndex - 1;

                if (previousIndex < 0)
                {
                    if (!cutChain.closed)
                    {
                        return false;
                    }

                    previousIndex =
                        vertexCount - 1;
                }

                if (previousIndex < 0 ||
                    previousIndex >=
                        static_cast<int>(
                            cutChain.halfEdgeIds.size()
                        ))
                {
                    return false;
                }

                const int forwardHalfEdgeId =
                    cutChain.halfEdgeIds[
                        previousIndex
                    ];

                if (forwardHalfEdgeId < 0 ||
                    forwardHalfEdgeId >=
                        static_cast<int>(
                            cutResult.mesh.halfEdges.size()
                        ))
                {
                    return false;
                }

                const int reverseHalfEdgeId =
                    cutResult.mesh.halfEdges[
                        forwardHalfEdgeId
                    ].twin;

                if (reverseHalfEdgeId < 0)
                {
                    return false;
                }

                boundaryCollection
                    .boundaryHalfEdgeIds
                    .insert(
                        reverseHalfEdgeId
                    );

                if (boundaryCollection.seedHalfEdgeId < 0)
                {
                    boundaryCollection.seedHalfEdgeId =
                        reverseHalfEdgeId;
                }

                currentIndex =
                    previousIndex;
            }
        }
    }

    return true;
}

Point3 subtractPoints(const Point3& first, const Point3& second)
{
    return {
        first.x - second.x,
        first.y - second.y,
        first.z - second.z
    };
}

Point3 addScaledPoint(
    const Point3& point,
    const Point3& direction,
    double scale
)
{
    return {
        point.x + direction.x * scale,
        point.y + direction.y * scale,
        point.z + direction.z * scale
    };
}

double dotPoints(const Point3& first, const Point3& second)
{
    return first.x * second.x +
        first.y * second.y +
        first.z * second.z;
}

double squaredDistance(const Point3& first, const Point3& second)
{
    const Point3 difference = subtractPoints(first, second);
    return dotPoints(difference, difference);
}

Point3 closestPointOnTriangle(
    const Point3& point,
    const TransferredRegionTriangle& triangle
)
{
    const Point3 firstSecond =
        subtractPoints(triangle.second, triangle.first);
    const Point3 firstThird =
        subtractPoints(triangle.third, triangle.first);
    const Point3 firstPoint =
        subtractPoints(point, triangle.first);
    const double d1 = dotPoints(firstSecond, firstPoint);
    const double d2 = dotPoints(firstThird, firstPoint);

    if (d1 <= 0.0 && d2 <= 0.0)
    {
        return triangle.first;
    }

    const Point3 secondPoint =
        subtractPoints(point, triangle.second);
    const double d3 = dotPoints(firstSecond, secondPoint);
    const double d4 = dotPoints(firstThird, secondPoint);

    if (d3 >= 0.0 && d4 <= d3)
    {
        return triangle.second;
    }

    const double vc = d1 * d4 - d3 * d2;

    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        return addScaledPoint(
            triangle.first,
            firstSecond,
            d1 / (d1 - d3)
        );
    }

    const Point3 thirdPoint =
        subtractPoints(point, triangle.third);
    const double d5 = dotPoints(firstSecond, thirdPoint);
    const double d6 = dotPoints(firstThird, thirdPoint);

    if (d6 >= 0.0 && d5 <= d6)
    {
        return triangle.third;
    }

    const double vb = d5 * d2 - d1 * d6;

    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        return addScaledPoint(
            triangle.first,
            firstThird,
            d2 / (d2 - d6)
        );
    }

    const double va = d3 * d6 - d5 * d4;

    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        const Point3 secondThird =
            subtractPoints(triangle.third, triangle.second);
        return addScaledPoint(
            triangle.second,
            secondThird,
            (d4 - d3) / ((d4 - d3) + (d5 - d6))
        );
    }

    const double denominator = 1.0 / (va + vb + vc);
    Point3 result = addScaledPoint(
        triangle.first,
        firstSecond,
        vb * denominator
    );
    return addScaledPoint(
        result,
        firstThird,
        vc * denominator
    );
}

Point3 meshFaceCentroid(const HalfEdgeMesh& mesh, int meshFaceId)
{
    const std::vector<int> halfEdgeIds = mesh.traverseFace(meshFaceId);
    Point3 centroid;

    if (halfEdgeIds.empty())
    {
        return centroid;
    }

    for (int halfEdgeId : halfEdgeIds)
    {
        const Point3& position = mesh.vertices[
            mesh.halfEdges[halfEdgeId].startVertex
        ].position;
        centroid.x += position.x;
        centroid.y += position.y;
        centroid.z += position.z;
    }

    const double inverseCount =
        1.0 / static_cast<double>(halfEdgeIds.size());
    centroid.x *= inverseCount;
    centroid.y *= inverseCount;
    centroid.z *= inverseCount;
    return centroid;
}
}

void CurvenetFaceRegionBuilder::build(
    CurvenetCutResult& cutResult
)
{
    for (CurvenetFace& curvenetFace :
         cutResult.curvenetFaces)
    {
        curvenetFace.meshFaceIds.clear();

        BoundaryCollection boundaryCollection;

        const bool boundaryCollected =
            collectBoundaryHalfEdges(
                curvenetFace,
                cutResult,
                boundaryCollection
            );

        if (!boundaryCollected)
        {
            continue;
        }

        if (
            boundaryCollection
                .boundaryHalfEdgeIds
                .empty()
        )
        {
            continue;
        }

        /*
            The oriented boundary half-edges follow the
            Curvenet face traversal. Their owning mesh face
            provides the starting face for the region.
        */
        const int firstBoundaryHalfEdgeId =
            boundaryCollection.seedHalfEdgeId;

        if (firstBoundaryHalfEdgeId < 0 ||
            firstBoundaryHalfEdgeId >=
                static_cast<int>(
                    cutResult.mesh.halfEdges.size()
                ))
        {
            continue;
        }

        const HalfEdge& seedBoundaryHalfEdge =
            cutResult.mesh.halfEdges[
                firstBoundaryHalfEdgeId
            ];

        const int seedTwinHalfEdgeId =
            seedBoundaryHalfEdge.twin;

        const auto floodFill =
            [&cutResult, &boundaryCollection](int seedFaceId)
            {
                std::unordered_set<int> visitedFaceIds;

                if (seedFaceId < 0 ||
                    seedFaceId >=
                        static_cast<int>(cutResult.mesh.faces.size()))
                {
                    return visitedFaceIds;
                }

                std::queue<int> pendingFaceIds;
                pendingFaceIds.push(seedFaceId);
                visitedFaceIds.insert(seedFaceId);

                while (!pendingFaceIds.empty())
                {
                    const int currentFaceId = pendingFaceIds.front();
                    pendingFaceIds.pop();

                    for (int currentHalfEdgeId :
                         cutResult.mesh.getFaceHalfEdges(currentFaceId))
                    {
                        const HalfEdge& currentHalfEdge =
                            cutResult.mesh.halfEdges[currentHalfEdgeId];
                        const int twinHalfEdgeId = currentHalfEdge.twin;

                        const bool boundaryEdge =
                            boundaryCollection.boundaryHalfEdgeIds.count(
                                currentHalfEdgeId
                            ) > 0 ||
                            boundaryCollection.boundaryHalfEdgeIds.count(
                                twinHalfEdgeId
                            ) > 0;

                        if (boundaryEdge || twinHalfEdgeId < 0 ||
                            twinHalfEdgeId >= static_cast<int>(
                                cutResult.mesh.halfEdges.size()
                            ))
                        {
                            continue;
                        }

                        const int neighbouringFaceId =
                            cutResult.mesh.halfEdges[twinHalfEdgeId].face;

                        if (neighbouringFaceId >= 0 &&
                            neighbouringFaceId < static_cast<int>(
                                cutResult.mesh.faces.size()
                            ) &&
                            visitedFaceIds.insert(neighbouringFaceId).second)
                        {
                            pendingFaceIds.push(neighbouringFaceId);
                        }
                    }
                }

                return visitedFaceIds;
            };

        const int orientedFaceId = seedBoundaryHalfEdge.face;
        const int oppositeFaceId =
            seedTwinHalfEdgeId >= 0 &&
                    seedTwinHalfEdgeId < static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    )
                ? cutResult.mesh.halfEdges[seedTwinHalfEdgeId].face
                : -1;

        std::unordered_set<int> visitedFaceIds =
            floodFill(oppositeFaceId);

        if (boundaryCollection.containsClosedCutChain)
        {
            if (visitedFaceIds.empty())
            {
                visitedFaceIds = floodFill(orientedFaceId);
            }
        }
        else
        {
            std::unordered_set<int> orientedRegion =
                floodFill(orientedFaceId);

            const auto boundaryCoverage = [
                &cutResult,
                &boundaryCollection
            ](const std::unordered_set<int>& region)
            {
                int coverage = 0;

                for (int boundaryHalfEdgeId :
                     boundaryCollection.boundaryHalfEdgeIds)
                {
                    if (boundaryHalfEdgeId < 0 ||
                        boundaryHalfEdgeId >= static_cast<int>(
                            cutResult.mesh.halfEdges.size()
                        ))
                    {
                        continue;
                    }

                    const HalfEdge& boundaryHalfEdge =
                        cutResult.mesh.halfEdges[boundaryHalfEdgeId];
                    const int twinHalfEdgeId = boundaryHalfEdge.twin;
                    const int oppositeBoundaryFaceId =
                        twinHalfEdgeId >= 0 &&
                        twinHalfEdgeId < static_cast<int>(
                            cutResult.mesh.halfEdges.size()
                        )
                            ? cutResult.mesh.halfEdges[
                                twinHalfEdgeId
                            ].face
                            : -1;

                    if (region.count(boundaryHalfEdge.face) > 0 ||
                        region.count(oppositeBoundaryFaceId) > 0)
                    {
                        ++coverage;
                    }
                }

                return coverage;
            };

            const int oppositeCoverage =
                boundaryCoverage(visitedFaceIds);
            const int orientedCoverage =
                boundaryCoverage(orientedRegion);

            if (visitedFaceIds.empty() ||
                (!orientedRegion.empty() &&
                 (
                    orientedCoverage > oppositeCoverage ||
                    (
                        orientedCoverage == oppositeCoverage &&
                        orientedRegion.size() < visitedFaceIds.size()
                    )
                 )))
            {
                visitedFaceIds = std::move(orientedRegion);
            }
        }

        curvenetFace.meshFaceIds.assign(
            visitedFaceIds.begin(),
            visitedFaceIds.end()
        );

        /*
            Keep the stored IDs deterministic for testing
            and later debugging.
        */
        std::sort(
            curvenetFace.meshFaceIds.begin(),
            curvenetFace.meshFaceIds.end()
        );
    }

    std::vector<std::unordered_set<int>>
        enclosedMeshFacesByCurvenetFace(
            cutResult.curvenetFaces.size()
        );

    for (int faceId = 0;
         faceId < static_cast<int>(cutResult.curvenetFaces.size());
         ++faceId)
    {
        enclosedMeshFacesByCurvenetFace[faceId].insert(
            cutResult.curvenetFaces[faceId].meshFaceIds.begin(),
            cutResult.curvenetFaces[faceId].meshFaceIds.end()
        );
    }

    /*
        Match logical faces to distinct physical components. Selecting each
        region from one boundary edge independently can seed the same side
        twice at a high-valence node and leave another closed cell unmapped.
    */
    std::vector<std::vector<int>> physicalComponents;
    std::vector<int> componentByMeshFace(
        cutResult.mesh.faces.size(),
        -1
    );
    std::unordered_set<int> physicalBarrierHalfEdgeIds =
        cutResult.embeddedHalfEdgeIds;

    for (const auto& chainEntry : cutResult.cutChainsByCurveId)
    {
        for (int halfEdgeId : chainEntry.second.halfEdgeIds)
        {
            if (halfEdgeId < 0 ||
                halfEdgeId >= static_cast<int>(
                    cutResult.mesh.halfEdges.size()
                ))
            {
                continue;
            }

            physicalBarrierHalfEdgeIds.insert(halfEdgeId);
            const int twinHalfEdgeId =
                cutResult.mesh.halfEdges[halfEdgeId].twin;

            if (twinHalfEdgeId >= 0)
            {
                physicalBarrierHalfEdgeIds.insert(twinHalfEdgeId);
            }
        }
    }

    for (int seedFaceId = 0;
         seedFaceId < static_cast<int>(cutResult.mesh.faces.size());
         ++seedFaceId)
    {
        if (componentByMeshFace[seedFaceId] >= 0)
        {
            continue;
        }

        const int componentId =
            static_cast<int>(physicalComponents.size());
        physicalComponents.emplace_back();
        std::queue<int> pendingFaceIds;
        pendingFaceIds.push(seedFaceId);
        componentByMeshFace[seedFaceId] = componentId;

        while (!pendingFaceIds.empty())
        {
            const int currentFaceId = pendingFaceIds.front();
            pendingFaceIds.pop();
            physicalComponents.back().push_back(currentFaceId);

            for (int halfEdgeId :
                 cutResult.mesh.getFaceHalfEdges(currentFaceId))
            {
                const int twinHalfEdgeId =
                    cutResult.mesh.halfEdges[halfEdgeId].twin;
                const bool boundaryEdge =
                    physicalBarrierHalfEdgeIds.count(halfEdgeId) > 0 ||
                    physicalBarrierHalfEdgeIds.count(twinHalfEdgeId) > 0;

                if (boundaryEdge || twinHalfEdgeId < 0 ||
                    twinHalfEdgeId >= static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    ))
                {
                    continue;
                }

                const int neighbouringFaceId =
                    cutResult.mesh.halfEdges[twinHalfEdgeId].face;

                if (neighbouringFaceId >= 0 &&
                    neighbouringFaceId < static_cast<int>(
                        cutResult.mesh.faces.size()
                    ) &&
                    componentByMeshFace[neighbouringFaceId] < 0)
                {
                    componentByMeshFace[neighbouringFaceId] = componentId;
                    pendingFaceIds.push(neighbouringFaceId);
                }
            }
        }

        std::sort(
            physicalComponents.back().begin(),
            physicalComponents.back().end()
        );
    }

    std::unordered_map<int, int> curveIdByHalfEdge;

    for (const auto& chainEntry : cutResult.cutChainsByCurveId)
    {
        for (int halfEdgeId : chainEntry.second.halfEdgeIds)
        {
            if (halfEdgeId < 0 ||
                halfEdgeId >= static_cast<int>(
                    cutResult.mesh.halfEdges.size()
                ))
            {
                continue;
            }

            curveIdByHalfEdge[halfEdgeId] = chainEntry.first;
            const int twinHalfEdgeId =
                cutResult.mesh.halfEdges[halfEdgeId].twin;

            if (twinHalfEdgeId >= 0)
            {
                curveIdByHalfEdge[twinHalfEdgeId] = chainEntry.first;
            }
        }
    }

    std::vector<std::unordered_set<int>> componentCurveIds(
        physicalComponents.size()
    );

    for (int componentId = 0;
         componentId < static_cast<int>(physicalComponents.size());
         ++componentId)
    {
        for (int meshFaceId : physicalComponents[componentId])
        {
            for (int halfEdgeId :
                 cutResult.mesh.getFaceHalfEdges(meshFaceId))
            {
                const auto curveIterator =
                    curveIdByHalfEdge.find(halfEdgeId);

                if (curveIterator != curveIdByHalfEdge.end())
                {
                    componentCurveIds[componentId].insert(
                        curveIterator->second
                    );
                }
            }
        }
    }

    std::vector<BoundaryCollection> faceBoundaries(
        cutResult.curvenetFaces.size()
    );
    std::vector<std::unordered_set<int>> faceCurveIds(
        cutResult.curvenetFaces.size()
    );
    std::vector<bool> validFaceBoundaries(
        cutResult.curvenetFaces.size(),
        false
    );

    for (int faceId = 0;
         faceId < static_cast<int>(cutResult.curvenetFaces.size());
         ++faceId)
    {
        validFaceBoundaries[faceId] = collectBoundaryHalfEdges(
            cutResult.curvenetFaces[faceId],
            cutResult,
            faceBoundaries[faceId]
        );

        for (const CurvenetFaceBoundary& boundary :
             cutResult.curvenetFaces[faceId].boundary)
        {
            faceCurveIds[faceId].insert(boundary.curveId);
        }

        cutResult.curvenetFaces[faceId].meshFaceIds.clear();
    }

    std::vector<bool> assignedFaces(
        cutResult.curvenetFaces.size(),
        false
    );
    std::vector<bool> assignedComponents(
        physicalComponents.size(),
        false
    );

    for (int assignmentIndex = 0;
         assignmentIndex < static_cast<int>(cutResult.curvenetFaces.size());
         ++assignmentIndex)
    {
        int bestFaceId = -1;
        int bestComponentId = -1;
        int bestMissingBoundaryEdges = std::numeric_limits<int>::max();
        int bestMissingCurves = std::numeric_limits<int>::max();
        int bestExtraCurves = std::numeric_limits<int>::max();
        int bestCoverage = -1;
        std::size_t bestComponentSize = 0;

        for (int faceId = 0;
             faceId < static_cast<int>(cutResult.curvenetFaces.size());
             ++faceId)
        {
            if (assignedFaces[faceId] || !validFaceBoundaries[faceId])
            {
                continue;
            }

            const BoundaryCollection& boundary = faceBoundaries[faceId];

            for (int componentId = 0;
                 componentId < static_cast<int>(physicalComponents.size());
                 ++componentId)
            {
                if (assignedComponents[componentId])
                {
                    continue;
                }

                int coverage = 0;

                for (int boundaryHalfEdgeId :
                     boundary.boundaryHalfEdgeIds)
                {
                    if (boundaryHalfEdgeId < 0 ||
                        boundaryHalfEdgeId >= static_cast<int>(
                            cutResult.mesh.halfEdges.size()
                        ))
                    {
                        continue;
                    }

                    const HalfEdge& boundaryHalfEdge =
                        cutResult.mesh.halfEdges[boundaryHalfEdgeId];
                    const int twinHalfEdgeId = boundaryHalfEdge.twin;
                    const int oppositeFaceId =
                        twinHalfEdgeId >= 0 &&
                                twinHalfEdgeId < static_cast<int>(
                                    cutResult.mesh.halfEdges.size()
                                )
                            ? cutResult.mesh.halfEdges[twinHalfEdgeId].face
                            : -1;

                    if ((boundaryHalfEdge.face >= 0 &&
                         componentByMeshFace[boundaryHalfEdge.face] ==
                             componentId) ||
                        (oppositeFaceId >= 0 &&
                         componentByMeshFace[oppositeFaceId] == componentId))
                    {
                        ++coverage;
                    }
                }

                int matchingCurves = 0;

                for (int curveId : faceCurveIds[faceId])
                {
                    if (componentCurveIds[componentId].count(curveId) > 0)
                    {
                        ++matchingCurves;
                    }
                }

                const int missingBoundaryEdges =
                    static_cast<int>(boundary.boundaryHalfEdgeIds.size()) -
                    coverage;
                const int missingCurves =
                    static_cast<int>(faceCurveIds[faceId].size()) -
                    matchingCurves;
                const int extraCurves =
                    static_cast<int>(componentCurveIds[componentId].size()) -
                    matchingCurves;
                const std::size_t componentSize =
                    physicalComponents[componentId].size();
                const bool betterCandidate =
                    missingBoundaryEdges < bestMissingBoundaryEdges ||
                    (missingBoundaryEdges == bestMissingBoundaryEdges &&
                     (missingCurves < bestMissingCurves ||
                      (missingCurves == bestMissingCurves &&
                       (extraCurves < bestExtraCurves ||
                        (extraCurves == bestExtraCurves &&
                         (coverage > bestCoverage ||
                          (coverage == bestCoverage &&
                           componentSize > bestComponentSize)))))));

                if (coverage > 0 && betterCandidate)
                {
                    bestFaceId = faceId;
                    bestComponentId = componentId;
                    bestMissingBoundaryEdges = missingBoundaryEdges;
                    bestMissingCurves = missingCurves;
                    bestExtraCurves = extraCurves;
                    bestCoverage = coverage;
                    bestComponentSize = componentSize;
                }
            }
        }

        if (bestFaceId < 0 || bestComponentId < 0)
        {
            break;
        }

        cutResult.curvenetFaces[bestFaceId].meshFaceIds =
            physicalComponents[bestComponentId];
        assignedFaces[bestFaceId] = true;
        assignedComponents[bestComponentId] = true;
    }

    /*
        Numerical junction slivers can split one logical Curvenet face into
        several physical components. Include every component enclosed by the
        logical face's own complete boundary instead of leaving it unmapped.
    */
    for (int componentId = 0;
         componentId < static_cast<int>(physicalComponents.size());
         ++componentId)
    {
        if (assignedComponents[componentId] ||
            physicalComponents[componentId].empty())
        {
            continue;
        }

        int bestFaceId = -1;
        std::size_t bestEnclosedRegionSize =
            std::numeric_limits<std::size_t>::max();

        for (int faceId = 0;
             faceId < static_cast<int>(cutResult.curvenetFaces.size());
             ++faceId)
        {
            const std::unordered_set<int>& enclosedMeshFaces =
                enclosedMeshFacesByCurvenetFace[faceId];
            const bool componentEnclosed = std::all_of(
                physicalComponents[componentId].begin(),
                physicalComponents[componentId].end(),
                [&enclosedMeshFaces](int meshFaceId)
                {
                    return enclosedMeshFaces.count(meshFaceId) > 0;
                }
            );

            if (componentEnclosed &&
                enclosedMeshFaces.size() < bestEnclosedRegionSize)
            {
                bestFaceId = faceId;
                bestEnclosedRegionSize = enclosedMeshFaces.size();
            }
        }

        if (bestFaceId < 0)
        {
            continue;
        }

        CurvenetFace& face = cutResult.curvenetFaces[bestFaceId];
        face.meshFaceIds.insert(
            face.meshFaceIds.end(),
            physicalComponents[componentId].begin(),
            physicalComponents[componentId].end()
        );
        assignedComponents[componentId] = true;
    }

    for (CurvenetFace& face : cutResult.curvenetFaces)
    {
        std::sort(face.meshFaceIds.begin(), face.meshFaceIds.end());
        face.meshFaceIds.erase(
            std::unique(
                face.meshFaceIds.begin(),
                face.meshFaceIds.end()
            ),
            face.meshFaceIds.end()
        );
    }
}

void CurvenetFaceRegionBuilder::buildFullSurfacePartitions(
    CurvenetCutResult& cutResult,
    int expectedRegionCount
)
{
    cutResult.curvenetFaces.clear();
    cutResult.fullSurfaceRegionCountBeforeCleanup = -1;
    cutResult.mergedFullSurfaceRegionPolygonCounts.clear();
    cutResult.mergedFullSurfaceRegionAreas.clear();

    std::unordered_set<int> visitedFaceIds;

    for (int seedFaceId = 0;
         seedFaceId < static_cast<int>(cutResult.mesh.faces.size());
         ++seedFaceId)
    {
        if (visitedFaceIds.count(seedFaceId) > 0)
        {
            continue;
        }

        CurvenetFace face;
        face.id = static_cast<int>(cutResult.curvenetFaces.size());

        std::queue<int> pendingFaceIds;
        pendingFaceIds.push(seedFaceId);
        visitedFaceIds.insert(seedFaceId);

        while (!pendingFaceIds.empty())
        {
            const int currentFaceId = pendingFaceIds.front();
            pendingFaceIds.pop();
            face.meshFaceIds.push_back(currentFaceId);

            for (int halfEdgeId :
                 cutResult.mesh.getFaceHalfEdges(currentFaceId))
            {
                if (halfEdgeId < 0 ||
                    halfEdgeId >= static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    ))
                {
                    continue;
                }

                const int twinHalfEdgeId =
                    cutResult.mesh.halfEdges[halfEdgeId].twin;

                const bool curvenetBarrier =
                    cutResult.embeddedHalfEdgeIds.count(halfEdgeId) > 0 ||
                    cutResult.embeddedHalfEdgeIds.count(twinHalfEdgeId) > 0;

                if (curvenetBarrier ||
                    twinHalfEdgeId < 0 ||
                    twinHalfEdgeId >= static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    ))
                {
                    continue;
                }

                const int neighbouringFaceId =
                    cutResult.mesh.halfEdges[twinHalfEdgeId].face;

                if (neighbouringFaceId >= 0 &&
                    neighbouringFaceId < static_cast<int>(
                        cutResult.mesh.faces.size()
                    ) &&
                    visitedFaceIds.insert(neighbouringFaceId).second)
                {
                    pendingFaceIds.push(neighbouringFaceId);
                }
            }
        }

        std::sort(face.meshFaceIds.begin(), face.meshFaceIds.end());

        cutResult.curvenetFaces.push_back(std::move(face));
    }

    const auto meshFaceArea = [&cutResult](int meshFaceId)
    {
        const std::vector<int> halfEdgeIds =
            cutResult.mesh.traverseFace(meshFaceId);

        if (halfEdgeIds.size() < 3)
        {
            return 0.0;
        }

        const Point3& anchor = cutResult.mesh.vertices[
            cutResult.mesh.halfEdges[halfEdgeIds[0]].startVertex
        ].position;
        double area = 0.0;

        for (int vertexIndex = 1;
             vertexIndex + 1 < static_cast<int>(halfEdgeIds.size());
             ++vertexIndex)
        {
            const Point3& first = cutResult.mesh.vertices[
                cutResult.mesh.halfEdges[
                    halfEdgeIds[vertexIndex]
                ].startVertex
            ].position;
            const Point3& second = cutResult.mesh.vertices[
                cutResult.mesh.halfEdges[
                    halfEdgeIds[vertexIndex + 1]
                ].startVertex
            ].position;
            const double firstX = first.x - anchor.x;
            const double firstY = first.y - anchor.y;
            const double firstZ = first.z - anchor.z;
            const double secondX = second.x - anchor.x;
            const double secondY = second.y - anchor.y;
            const double secondZ = second.z - anchor.z;
            const double crossX = firstY * secondZ - firstZ * secondY;
            const double crossY = firstZ * secondX - firstX * secondZ;
            const double crossZ = firstX * secondY - firstY * secondX;

            area += 0.5 * std::sqrt(
                crossX * crossX + crossY * crossY + crossZ * crossZ
            );
        }

        return area;
    };

    std::vector<double> regionAreas(
        cutResult.curvenetFaces.size(),
        0.0
    );
    double totalArea = 0.0;

    for (int regionId = 0;
         regionId < static_cast<int>(cutResult.curvenetFaces.size());
         ++regionId)
    {
        for (int meshFaceId :
             cutResult.curvenetFaces[regionId].meshFaceIds)
        {
            regionAreas[regionId] += meshFaceArea(meshFaceId);
        }

        totalArea += regionAreas[regionId];
    }

    cutResult.fullSurfaceRegionCountBeforeCleanup =
        static_cast<int>(cutResult.curvenetFaces.size());

    /*
        Compare against a typical cut polygon, not the entire character.
        Otherwise a valid region on a small feature such as a finger can
        look numerically negligible relative to the full body surface.
    */
    const double meanMeshFaceArea =
        cutResult.mesh.faces.empty()
            ? 0.0
            : totalArea /
                static_cast<double>(cutResult.mesh.faces.size());
    const double strictDegenerateAreaTolerance =
        meanMeshFaceArea * 0.001;
    const double smallCellAreaTolerance =
        meanMeshFaceArea * 0.01;
    const auto isDegenerateRegion = [
        &regionAreas,
        &cutResult,
        strictDegenerateAreaTolerance,
        smallCellAreaTolerance
    ](int regionId)
    {
        const std::size_t polygonCount =
            cutResult.curvenetFaces[regionId].meshFaceIds.size();

        return
            regionAreas[regionId] <= strictDegenerateAreaTolerance ||
            (
                polygonCount <= 3 &&
                regionAreas[regionId] <= smallCellAreaTolerance
            );
    };
    std::vector<int> regionByMeshFace(
        cutResult.mesh.faces.size(),
        -1
    );

    for (int regionId = 0;
         regionId < static_cast<int>(cutResult.curvenetFaces.size());
         ++regionId)
    {
        for (int meshFaceId :
             cutResult.curvenetFaces[regionId].meshFaceIds)
        {
            regionByMeshFace[meshFaceId] = regionId;
        }
    }

    std::vector<int> mergeTargetByRegion(
        cutResult.curvenetFaces.size(),
        -1
    );

    for (int regionId = 0;
         regionId < static_cast<int>(cutResult.curvenetFaces.size());
         ++regionId)
    {
        if (!isDegenerateRegion(regionId))
        {
            continue;
        }

        std::unordered_map<int, double> sharedBoundaryLength;

        for (int meshFaceId :
             cutResult.curvenetFaces[regionId].meshFaceIds)
        {
            for (int halfEdgeId :
                 cutResult.mesh.getFaceHalfEdges(meshFaceId))
            {
                const HalfEdge& halfEdge =
                    cutResult.mesh.halfEdges[halfEdgeId];

                if (halfEdge.twin < 0 ||
                    halfEdge.twin >= static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    ))
                {
                    continue;
                }

                /* Never remove an authored Curvenet boundary while cleaning
                   a numerical sliver beside an almost coincident mesh edge. */
                if (cutResult.embeddedHalfEdgeIds.count(halfEdgeId) > 0 ||
                    cutResult.embeddedHalfEdgeIds.count(halfEdge.twin) > 0)
                {
                    continue;
                }

                const int neighbouringFaceId =
                    cutResult.mesh.halfEdges[halfEdge.twin].face;

                if (neighbouringFaceId < 0 ||
                    neighbouringFaceId >= static_cast<int>(
                        regionByMeshFace.size()
                    ))
                {
                    continue;
                }

                const int neighbouringRegionId =
                    regionByMeshFace[neighbouringFaceId];

                if (neighbouringRegionId < 0 ||
                    neighbouringRegionId == regionId ||
                    isDegenerateRegion(neighbouringRegionId))
                {
                    continue;
                }

                const Point3& start =
                    cutResult.mesh.vertices[
                        halfEdge.startVertex
                    ].position;
                const Point3& end =
                    cutResult.mesh.vertices[
                        halfEdge.endVertex
                    ].position;

                sharedBoundaryLength[neighbouringRegionId] +=
                    GeometryUtils::pointToPointDistance(start, end);
            }
        }

        double longestBoundary = -std::numeric_limits<double>::infinity();

        for (const auto& entry : sharedBoundaryLength)
        {
            if (entry.second > longestBoundary)
            {
                longestBoundary = entry.second;
                mergeTargetByRegion[regionId] = entry.first;
            }
        }
    }

    if (expectedRegionCount >= 0)
    {
        const int surplusRegionCount = std::max(
            0,
            static_cast<int>(cutResult.curvenetFaces.size()) -
                expectedRegionCount
        );
        std::vector<int> mergeCandidates;

        for (int regionId = 0;
             regionId < static_cast<int>(mergeTargetByRegion.size());
             ++regionId)
        {
            if (mergeTargetByRegion[regionId] >= 0)
            {
                mergeCandidates.push_back(regionId);
            }
        }

        std::sort(
            mergeCandidates.begin(),
            mergeCandidates.end(),
            [&regionAreas](int first, int second)
            {
                return regionAreas[first] < regionAreas[second];
            }
        );

        for (int candidateIndex = surplusRegionCount;
             candidateIndex < static_cast<int>(mergeCandidates.size());
             ++candidateIndex)
        {
            mergeTargetByRegion[mergeCandidates[candidateIndex]] = -1;
        }

        int selectedMergeCount = std::min(
            surplusRegionCount,
            static_cast<int>(mergeCandidates.size())
        );
        std::vector<bool> selectedForMerge(
            cutResult.curvenetFaces.size(),
            false
        );
        std::vector<bool> protectedMergeTargets(
            cutResult.curvenetFaces.size(),
            false
        );

        for (int regionId = 0;
             regionId < static_cast<int>(mergeTargetByRegion.size());
             ++regionId)
        {
            const int targetRegionId = mergeTargetByRegion[regionId];

            if (targetRegionId >= 0)
            {
                selectedForMerge[regionId] = true;
                protectedMergeTargets[targetRegionId] = true;
            }
        }

        std::vector<int> remainingCandidates(
            cutResult.curvenetFaces.size()
        );

        for (int regionId = 0;
             regionId < static_cast<int>(remainingCandidates.size());
             ++regionId)
        {
            remainingCandidates[regionId] = regionId;
        }

        std::sort(
            remainingCandidates.begin(),
            remainingCandidates.end(),
            [&regionAreas](int first, int second)
            {
                return regionAreas[first] < regionAreas[second];
            }
        );

        for (int regionId : remainingCandidates)
        {
            if (selectedMergeCount >= surplusRegionCount)
            {
                break;
            }

            if (selectedForMerge[regionId] ||
                protectedMergeTargets[regionId])
            {
                continue;
            }

            std::unordered_map<int, double> sharedBoundaryLength;

            for (int meshFaceId :
                 cutResult.curvenetFaces[regionId].meshFaceIds)
            {
                for (int halfEdgeId :
                     cutResult.mesh.getFaceHalfEdges(meshFaceId))
                {
                    const HalfEdge& halfEdge =
                        cutResult.mesh.halfEdges[halfEdgeId];

                    if (halfEdge.twin < 0 ||
                        halfEdge.twin >= static_cast<int>(
                            cutResult.mesh.halfEdges.size()
                        ))
                    {
                        continue;
                    }

                    if (cutResult.embeddedHalfEdgeIds.count(halfEdgeId) > 0 ||
                        cutResult.embeddedHalfEdgeIds.count(halfEdge.twin) > 0)
                    {
                        continue;
                    }

                    const int neighbouringFaceId =
                        cutResult.mesh.halfEdges[halfEdge.twin].face;

                    if (neighbouringFaceId < 0 ||
                        neighbouringFaceId >= static_cast<int>(
                            regionByMeshFace.size()
                        ))
                    {
                        continue;
                    }

                    const int neighbouringRegionId =
                        regionByMeshFace[neighbouringFaceId];

                    if (neighbouringRegionId < 0 ||
                        neighbouringRegionId == regionId ||
                        selectedForMerge[neighbouringRegionId])
                    {
                        continue;
                    }

                    const Point3& start = cutResult.mesh.vertices[
                        halfEdge.startVertex
                    ].position;
                    const Point3& end = cutResult.mesh.vertices[
                        halfEdge.endVertex
                    ].position;
                    sharedBoundaryLength[neighbouringRegionId] +=
                        GeometryUtils::pointToPointDistance(start, end);
                }
            }

            int targetRegionId = -1;
            double longestBoundary =
                -std::numeric_limits<double>::infinity();

            for (const auto& entry : sharedBoundaryLength)
            {
                if (entry.second > longestBoundary)
                {
                    targetRegionId = entry.first;
                    longestBoundary = entry.second;
                }
            }

            if (targetRegionId < 0)
            {
                continue;
            }

            mergeTargetByRegion[regionId] = targetRegionId;
            selectedForMerge[regionId] = true;
            protectedMergeTargets[targetRegionId] = true;
            ++selectedMergeCount;
        }
    }

    for (int regionId = 0;
         regionId < static_cast<int>(mergeTargetByRegion.size());
         ++regionId)
    {
        const int targetRegionId = mergeTargetByRegion[regionId];

        if (targetRegionId < 0)
        {
            continue;
        }

        cutResult.mergedFullSurfaceRegionPolygonCounts.push_back(
            static_cast<int>(
                cutResult.curvenetFaces[regionId].meshFaceIds.size()
            )
        );
        cutResult.mergedFullSurfaceRegionAreas.push_back(
            regionAreas[regionId]
        );

        CurvenetFace& targetFace =
            cutResult.curvenetFaces[targetRegionId];
        const CurvenetFace& sourceFace =
            cutResult.curvenetFaces[regionId];
        targetFace.meshFaceIds.insert(
            targetFace.meshFaceIds.end(),
            sourceFace.meshFaceIds.begin(),
            sourceFace.meshFaceIds.end()
        );
    }

    std::vector<CurvenetFace> nonDegenerateFaces;

    for (int regionId = 0;
         regionId < static_cast<int>(cutResult.curvenetFaces.size());
         ++regionId)
    {
        if (mergeTargetByRegion[regionId] >= 0)
        {
            continue;
        }

        CurvenetFace face =
            std::move(cutResult.curvenetFaces[regionId]);
        std::sort(face.meshFaceIds.begin(), face.meshFaceIds.end());
        face.meshFaceIds.erase(
            std::unique(face.meshFaceIds.begin(), face.meshFaceIds.end()),
            face.meshFaceIds.end()
        );
        face.id = static_cast<int>(nonDegenerateFaces.size());
        nonDegenerateFaces.push_back(std::move(face));
    }

    cutResult.curvenetFaces = std::move(nonDegenerateFaces);
}

void CurvenetFaceRegionBuilder::buildAuthoredSurfacePartitions(
    CurvenetCutResult& cutResult,
    int expectedFaceCount
)
{
    buildFullSurfacePartitions(
        cutResult,
        expectedFaceCount >= 0 ? expectedFaceCount + 1 : -1
    );

    if (cutResult.curvenetFaces.empty())
    {
        return;
    }

    std::vector<int> boundaryEdgeCountByRegion(
        cutResult.curvenetFaces.size(),
        0
    );
    std::vector<int> regionByMeshFace(
        cutResult.mesh.faces.size(),
        -1
    );

    for (int regionId = 0;
         regionId < static_cast<int>(cutResult.curvenetFaces.size());
         ++regionId)
    {
        for (int meshFaceId :
             cutResult.curvenetFaces[regionId].meshFaceIds)
        {
            if (meshFaceId >= 0 &&
                meshFaceId < static_cast<int>(regionByMeshFace.size()))
            {
                regionByMeshFace[meshFaceId] = regionId;
            }
        }
    }

    for (const HalfEdge& halfEdge : cutResult.mesh.halfEdges)
    {
        if (halfEdge.twin >= 0 ||
            halfEdge.face < 0 ||
            halfEdge.face >= static_cast<int>(regionByMeshFace.size()))
        {
            continue;
        }

        const int regionId = regionByMeshFace[halfEdge.face];

        if (regionId >= 0)
        {
            ++boundaryEdgeCountByRegion[regionId];
        }
    }

    int exteriorRegionId = -1;
    int largestBoundaryEdgeCount = 0;

    for (int regionId = 0;
         regionId < static_cast<int>(boundaryEdgeCountByRegion.size());
         ++regionId)
    {
        if (boundaryEdgeCountByRegion[regionId] > largestBoundaryEdgeCount)
        {
            largestBoundaryEdgeCount = boundaryEdgeCountByRegion[regionId];
            exteriorRegionId = regionId;
        }
    }

    if (exteriorRegionId < 0)
    {
        exteriorRegionId = static_cast<int>(std::distance(
            cutResult.curvenetFaces.begin(),
            std::max_element(
                cutResult.curvenetFaces.begin(),
                cutResult.curvenetFaces.end(),
                [](const CurvenetFace& first, const CurvenetFace& second)
                {
                    return first.meshFaceIds.size() <
                        second.meshFaceIds.size();
                }
            )
        ));
    }

    if (expectedFaceCount < 0 ||
        static_cast<int>(cutResult.curvenetFaces.size()) > expectedFaceCount)
    {
        cutResult.curvenetFaces.erase(
            cutResult.curvenetFaces.begin() + exteriorRegionId
        );
    }

    for (int faceId = 0;
         faceId < static_cast<int>(cutResult.curvenetFaces.size());
         ++faceId)
    {
        cutResult.curvenetFaces[faceId].id = faceId;
    }
}

void CurvenetFaceRegionBuilder::buildTransferredLogicalPartitions(
    CurvenetCutResult& cutResult,
    const std::vector<TransferredRegionTriangle>& sourceTriangles,
    int expectedFaceCount
)
{
    if (sourceTriangles.empty())
    {
        buildFullSurfacePartitions(cutResult, expectedFaceCount);
        return;
    }

    int sourceRegionCount = 0;

    for (const TransferredRegionTriangle& triangle : sourceTriangles)
    {
        sourceRegionCount = std::max(
            sourceRegionCount,
            triangle.regionId + 1
        );
    }

    const int regionCount = std::max(
        sourceRegionCount,
        expectedFaceCount
    );

    if (regionCount <= 0)
    {
        buildFullSurfacePartitions(cutResult, expectedFaceCount);
        return;
    }

    cutResult.curvenetFaces.clear();
    cutResult.curvenetFaces.resize(regionCount);

    for (int regionId = 0; regionId < regionCount; ++regionId)
    {
        cutResult.curvenetFaces[regionId].id = regionId;
    }

    std::vector<Point3> meshFaceCentroids;
    meshFaceCentroids.reserve(cutResult.mesh.faces.size());

    for (int meshFaceId = 0;
         meshFaceId < static_cast<int>(cutResult.mesh.faces.size());
         ++meshFaceId)
    {
        meshFaceCentroids.push_back(
            meshFaceCentroid(cutResult.mesh, meshFaceId)
        );
    }

    for (int meshFaceId = 0;
         meshFaceId < static_cast<int>(cutResult.mesh.faces.size());
         ++meshFaceId)
    {
        const Point3& centroid = meshFaceCentroids[meshFaceId];
        int closestRegionId = -1;
        double closestSquaredDistance =
            std::numeric_limits<double>::infinity();

        for (const TransferredRegionTriangle& triangle : sourceTriangles)
        {
            if (triangle.regionId < 0 ||
                triangle.regionId >= regionCount)
            {
                continue;
            }

            const Point3 closest =
                closestPointOnTriangle(centroid, triangle);
            const double distance =
                squaredDistance(centroid, closest);

            if (distance < closestSquaredDistance)
            {
                closestSquaredDistance = distance;
                closestRegionId = triangle.regionId;
            }
        }

        if (closestRegionId >= 0)
        {
            cutResult.curvenetFaces[closestRegionId]
                .meshFaceIds.push_back(meshFaceId);
        }
    }

    /*
        A narrow source region may contain no target face centroid on a
        coarser topology. Preserve every transferred logical region by
        assigning its closest available target face.
    */
    std::unordered_set<int> recoveredMeshFaceIds;

    for (int missingRegionId = 0;
         missingRegionId < regionCount;
         ++missingRegionId)
    {
        if (!cutResult.curvenetFaces[missingRegionId]
                 .meshFaceIds.empty())
        {
            continue;
        }

        int closestMeshFaceId = -1;
        int donorRegionId = -1;
        double closestSquaredDistance =
            std::numeric_limits<double>::infinity();

        for (int candidateRegionId = 0;
             candidateRegionId < regionCount;
             ++candidateRegionId)
        {
            const std::vector<int>& candidateMeshFaceIds =
                cutResult.curvenetFaces[candidateRegionId]
                    .meshFaceIds;

            if (candidateMeshFaceIds.size() <= 1)
            {
                continue;
            }

            for (int meshFaceId : candidateMeshFaceIds)
            {
                if (recoveredMeshFaceIds.count(meshFaceId) > 0)
                {
                    continue;
                }

                for (const TransferredRegionTriangle& triangle :
                     sourceTriangles)
                {
                    if (triangle.regionId != missingRegionId)
                    {
                        continue;
                    }

                    const Point3 closest = closestPointOnTriangle(
                        meshFaceCentroids[meshFaceId],
                        triangle
                    );
                    const double distance = squaredDistance(
                        meshFaceCentroids[meshFaceId],
                        closest
                    );

                    if (distance < closestSquaredDistance)
                    {
                        closestSquaredDistance = distance;
                        closestMeshFaceId = meshFaceId;
                        donorRegionId = candidateRegionId;
                    }
                }
            }
        }

        if (closestMeshFaceId < 0 || donorRegionId < 0)
        {
            continue;
        }

        std::vector<int>& donorMeshFaceIds =
            cutResult.curvenetFaces[donorRegionId].meshFaceIds;
        donorMeshFaceIds.erase(
            std::find(
                donorMeshFaceIds.begin(),
                donorMeshFaceIds.end(),
                closestMeshFaceId
            )
        );
        cutResult.curvenetFaces[missingRegionId]
            .meshFaceIds.push_back(closestMeshFaceId);
        recoveredMeshFaceIds.insert(closestMeshFaceId);
    }

    cutResult.curvenetFaces.erase(
        std::remove_if(
            cutResult.curvenetFaces.begin(),
            cutResult.curvenetFaces.end(),
            [](const CurvenetFace& face)
            {
                return face.meshFaceIds.empty();
            }
        ),
        cutResult.curvenetFaces.end()
    );

    for (int faceId = 0;
         faceId < static_cast<int>(cutResult.curvenetFaces.size());
         ++faceId)
    {
        cutResult.curvenetFaces[faceId].id = faceId;
    }

    cutResult.fullSurfaceRegionCountBeforeCleanup =
        static_cast<int>(cutResult.curvenetFaces.size());
    cutResult.mergedFullSurfaceRegionPolygonCounts.clear();
    cutResult.mergedFullSurfaceRegionAreas.clear();
}
