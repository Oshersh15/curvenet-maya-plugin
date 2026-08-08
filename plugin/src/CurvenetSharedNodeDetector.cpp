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

        for (
            const EmbeddedCurvePoint& point :
            cutChain.points
        )
        {
            const int meshVertexId =
                point.meshVertexId;

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
                continue;
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
        }
    }

    return std::nullopt;
}
