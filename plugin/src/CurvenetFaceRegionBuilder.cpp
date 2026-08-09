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
                            ) > 0 ||
                            cutResult.embeddedHalfEdgeIds.count(
                                currentHalfEdgeId
                            ) > 0 ||
                            cutResult.embeddedHalfEdgeIds.count(
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

            if (visitedFaceIds.empty() ||
                (!orientedRegion.empty() &&
                 orientedRegion.size() < visitedFaceIds.size()))
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
}

void CurvenetFaceRegionBuilder::buildFullSurfacePartitions(
    CurvenetCutResult& cutResult
)
{
    cutResult.curvenetFaces.clear();

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

    const double degenerateAreaTolerance =
        totalArea * 0.0001;
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
        if (regionAreas[regionId] > degenerateAreaTolerance)
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
                    regionAreas[neighbouringRegionId] <=
                        degenerateAreaTolerance)
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

    for (int regionId = 0;
         regionId < static_cast<int>(mergeTargetByRegion.size());
         ++regionId)
    {
        const int targetRegionId = mergeTargetByRegion[regionId];

        if (targetRegionId < 0)
        {
            continue;
        }

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
