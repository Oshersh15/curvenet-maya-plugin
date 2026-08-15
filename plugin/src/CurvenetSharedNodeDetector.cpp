/* Resolves physical or authored endpoints into shared logical Curvenet nodes. */

#include "CurvenetSharedNodeDetector.h"

#include "GeometryUtils.h"

std::optional<int>
CurvenetSharedNodeDetector::findSharedMeshVertex(
    const CutVertex& endpoint,
    const CurvenetCutResult& curvenetResult,
    double positionTolerance
)
{
    for (const auto& entry :
         curvenetResult.cutChainsByCurveId)
    {
        const CutChain& cutChain =
            entry.second;

        const auto matchingMeshVertex =
            [&](int meshVertexId) -> std::optional<int>
            {
                if (
                    meshVertexId < 0 ||
                    meshVertexId >=
                        static_cast<int>(
                            curvenetResult
                                .mesh
                                .vertices
                                .size()
                        )
                )
                {
                    return std::nullopt;
                }

                const Point3& existingPosition =
                    curvenetResult
                        .mesh
                        .vertices[
                            meshVertexId
                        ]
                        .position;

                const double positionDistance =
                    GeometryUtils::pointToPointDistance(
                        endpoint.position,
                        existingPosition
                    );

                if (
                    positionDistance <=
                    positionTolerance
                )
                {
                    return meshVertexId;
                }

                return std::nullopt;
            };

        for (const EmbeddedCurvePoint& point :
             cutChain.points)
        {
            const std::optional<int> match =
                matchingMeshVertex(
                    point.meshVertexId
                );

            if (match.has_value())
            {
                return match;
            }
        }

        /*
        Legacy and manually constructed CutChains may only provide vertexIds.
        */
        if (cutChain.points.empty())
        {
            for (int meshVertexId :
                 cutChain.vertexIds)
            {
                const std::optional<int> match =
                    matchingMeshVertex(
                        meshVertexId
                    );

                if (match.has_value())
                {
                    return match;
                }
            }
        }
    }

    return std::nullopt;
}
