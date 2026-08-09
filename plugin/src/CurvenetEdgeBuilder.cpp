#include "CurvenetEdgeBuilder.h"
#include "CurvenetCutResult.h"
#include "GeometryUtils.h"
#include "ProfileCutInput.h"

#include <maya/MGlobal.h>
#include <maya/MString.h>

#include <algorithm>

void CurvenetEdgeBuilder::build(
    CurvenetCutResult& curvenetCutResult,
    const std::vector<ProfileCutInput>& profileInputs
)

{
    curvenetCutResult.curvenetEdges.clear();

    int nextEdgeId = 0;

    for (
        const auto& entry :
        curvenetCutResult.cutChainsByCurveId
    )
    {
        const CutChain& cutChain =
            entry.second;

        const ProfileCutInput* profileInput = nullptr;

        for (const ProfileCutInput& input : profileInputs)
        {
            if (input.curveId == cutChain.curveId)
            {
                profileInput = &input;
                break;
            }
        }

        if (profileInput == nullptr)
        {
            continue;
        }

        if (cutChain.vertexIds.empty())
        {
            continue;
        }

        std::vector<int> sharedNodes;
        std::vector<int> sharedPointIndices;

        for (
            size_t i = 0;
            i <
            curvenetCutResult.sharedCurvenetNodes.size();
            ++i
        )
        {
            const SharedCurvenetNode& node =
                curvenetCutResult
                    .sharedCurvenetNodes[i];

            for (
                size_t pointIndex = 0;
                pointIndex < cutChain.points.size();
                ++pointIndex
            )
            {
                const EmbeddedCurvePoint& point =
                    cutChain.points[pointIndex];

                if (
                    point.meshVertexId ==
                    node.meshVertexId
                )
                {
                    sharedNodes.push_back(
                        static_cast<int>(i)
                    );

                    sharedPointIndices.push_back(
                        static_cast<int>(pointIndex)
                    );
                }
            }

            if (node.meshVertexId < 0 &&
                std::find(
                    node.connectedCurveIds.begin(),
                    node.connectedCurveIds.end(),
                    cutChain.curveId
                ) != node.connectedCurveIds.end() &&
                !cutChain.points.empty() &&
                !profileInput->sampledSegments.empty())
            {
                const Point3& startPosition =
                    profileInput->sampledSegments.front().start;
                const Point3& endPosition =
                    profileInput->sampledSegments.back().end;
                const double startDistance =
                    GeometryUtils::pointToPointDistance(
                        node.position,
                        startPosition
                    );
                const double endDistance =
                    GeometryUtils::pointToPointDistance(
                        node.position,
                        endPosition
                    );

                sharedNodes.push_back(static_cast<int>(i));
                sharedPointIndices.push_back(
                    startDistance <= endDistance
                        ? 0
                        : static_cast<int>(cutChain.points.size()) - 1
                );
            }
        }

        if (sharedNodes.size() < 2)
        {
            continue;
        }

        std::vector<int> orderedIndices(
            sharedPointIndices.size(),
            0
        );

        for (size_t i = 0;
             i < orderedIndices.size();
             ++i)
        {
            orderedIndices[i] =
                static_cast<int>(i);
        }

        std::sort(
            orderedIndices.begin(),
            orderedIndices.end(),
            [&sharedPointIndices](
                int first,
                int second
            )
            {
                return sharedPointIndices[first] <
                       sharedPointIndices[second];
            }
        );

        const int startSharedIndex =
            orderedIndices.front();

        const int endSharedIndex =
            orderedIndices.back();

        CurvenetEdge edge;

        edge.id =
            nextEdgeId++;

        edge.startNode =
            sharedNodes[startSharedIndex];

        edge.endNode =
            sharedNodes[endSharedIndex];

        edge.sourceCurveId =
            cutChain.curveId;

        if (!profileInput->sampledSegments.empty())
        {
            edge.sampledPoints.push_back(
                profileInput->sampledSegments.front().start
            );

            for (
                const PolylineSegment& segment :
                profileInput->sampledSegments
            )
            {
                edge.sampledPoints.push_back(
                    segment.end
                );
            }
        }

        curvenetCutResult
            .curvenetEdges
            .push_back(edge);
    }

    MGlobal::displayInfo(
        MString("Curvenet edges: ")
        + static_cast<int>(
            curvenetCutResult
                .curvenetEdges
                .size()
        )
    );
}
