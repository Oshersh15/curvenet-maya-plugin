#include "CurvenetFaceRegionBuilder.h"

#include <unordered_set>
#include <algorithm>
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
