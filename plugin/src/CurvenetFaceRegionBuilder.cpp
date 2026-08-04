#include "CurvenetFaceRegionBuilder.h"

#include <unordered_set>
#include <algorithm>
#include <queue>

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

        /*
            Prefer the face on the opposite side of the
            oriented boundary half-edge.

            Some valid test and boundary-mesh cases do not
            provide a twin with a valid owning face. In that
            case, fall back to the oriented half-edge's face.
        */
        int seedFaceId = -1;

        const HalfEdge& seedBoundaryHalfEdge =
            cutResult.mesh.halfEdges[
                firstBoundaryHalfEdgeId
            ];

        const int seedTwinHalfEdgeId =
            seedBoundaryHalfEdge.twin;

        if (
            seedTwinHalfEdgeId >= 0 &&
            seedTwinHalfEdgeId <
                static_cast<int>(
                    cutResult.mesh.halfEdges.size()
                )
        )
        {
            const int twinFaceId =
                cutResult.mesh.halfEdges[
                    seedTwinHalfEdgeId
                ].face;

            if (
                twinFaceId >= 0 &&
                twinFaceId <
                    static_cast<int>(
                        cutResult.mesh.faces.size()
                    )
            )
            {
                seedFaceId =
                    twinFaceId;
            }
        }

        if (seedFaceId < 0)
        {
            seedFaceId =
                seedBoundaryHalfEdge.face;
        }

        if (
            seedFaceId < 0 ||
            seedFaceId >=
                static_cast<int>(
                    cutResult.mesh.faces.size()
                )
        )
        {
            continue;
        }

        std::queue<int> pendingFaceIds;
        std::unordered_set<int> visitedFaceIds;

        pendingFaceIds.push(seedFaceId);
        visitedFaceIds.insert(seedFaceId);

        while (!pendingFaceIds.empty())
        {
            const int currentFaceId =
                pendingFaceIds.front();

            pendingFaceIds.pop();

            const int startingHalfEdgeId =
                cutResult.mesh.faces[
                    currentFaceId
                ].halfEdge;

            if (startingHalfEdgeId < 0 ||
                startingHalfEdgeId >=
                    static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    ))
            {
                continue;
            }

            int currentHalfEdgeId =
                startingHalfEdgeId;

            do
            {
                if (currentHalfEdgeId < 0 ||
                    currentHalfEdgeId >=
                        static_cast<int>(
                            cutResult.mesh.halfEdges.size()
                        ))
                {
                    break;
                }

                const HalfEdge& currentHalfEdge =
                    cutResult.mesh.halfEdges[
                        currentHalfEdgeId
                    ];

                const int twinHalfEdgeId =
                    currentHalfEdge.twin;

                /*
                    Do not cross a Curvenet boundary edge.

                    Check both directions because the current
                    mesh face may reference either half-edge
                    of the same physical boundary edge.
                */
                const bool boundaryEdge =
                    boundaryCollection.boundaryHalfEdgeIds.find(
                        currentHalfEdgeId
                    ) != boundaryCollection.boundaryHalfEdgeIds.end() ||
                    boundaryCollection.boundaryHalfEdgeIds.find(
                        twinHalfEdgeId
                    ) != boundaryCollection.boundaryHalfEdgeIds.end();

                if (!boundaryEdge &&
                    twinHalfEdgeId >= 0 &&
                    twinHalfEdgeId <
                        static_cast<int>(
                            cutResult.mesh.halfEdges.size()
                        ))
                {
                    const int neighbouringFaceId =
                        cutResult.mesh.halfEdges[
                            twinHalfEdgeId
                        ].face;

                    if (neighbouringFaceId >= 0 &&
                        neighbouringFaceId <
                            static_cast<int>(
                                cutResult.mesh.faces.size()
                            ) &&
                        visitedFaceIds.insert(
                            neighbouringFaceId
                        ).second)
                    {
                        pendingFaceIds.push(
                            neighbouringFaceId
                        );
                    }
                }

                currentHalfEdgeId =
                    currentHalfEdge.next;
            }
            while (
                currentHalfEdgeId !=
                startingHalfEdgeId
            );
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
