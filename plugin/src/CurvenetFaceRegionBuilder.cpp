#include "CurvenetFaceRegionBuilder.h"

#include <unordered_set>
#include <algorithm>
#include <queue>

namespace
{
bool collectBoundaryHalfEdges(
    const CurvenetFace& curvenetFace,
    const CurvenetCutResult& cutResult,
    std::unordered_set<int>& boundaryHalfEdgeIds
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

                boundaryHalfEdgeIds.insert(
                    cutChain.halfEdgeIds[
                        currentIndex
                    ]
                );

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

                boundaryHalfEdgeIds.insert(
                    reverseHalfEdgeId
                );

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

        std::unordered_set<int>
            boundaryHalfEdgeIds;

        const bool boundaryCollected =
            collectBoundaryHalfEdges(
                curvenetFace,
                cutResult,
                boundaryHalfEdgeIds
            );

        if (!boundaryCollected)
        {
            continue;
        }

        if (boundaryHalfEdgeIds.empty())
        {
            continue;
        }

        /*
            The oriented boundary half-edges follow the
            Curvenet face traversal. Their owning mesh face
            provides the starting face for the region.
        */
        const int firstBoundaryHalfEdgeId =
            *boundaryHalfEdgeIds.begin();

        if (firstBoundaryHalfEdgeId < 0 ||
            firstBoundaryHalfEdgeId >=
                static_cast<int>(
                    cutResult.mesh.halfEdges.size()
                ))
        {
            continue;
        }

        const int seedFaceId =
            cutResult.mesh.halfEdges[
                firstBoundaryHalfEdgeId
            ].face;

        if (seedFaceId < 0 ||
            seedFaceId >=
                static_cast<int>(
                    cutResult.mesh.faces.size()
                ))
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
                    boundaryHalfEdgeIds.find(
                        currentHalfEdgeId
                    ) != boundaryHalfEdgeIds.end() ||
                    boundaryHalfEdgeIds.find(
                        twinHalfEdgeId
                    ) != boundaryHalfEdgeIds.end();

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
