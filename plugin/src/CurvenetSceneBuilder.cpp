#include "CurvenetSceneBuilder.h"
#include "CurvenetEdge.h"
#include "CurvenetRegionPreviewBuilder.h"

#include <maya/MGlobal.h>
#include <maya/MString.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MPointArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MObject.h>

#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

int findSharedNodeIndex(
    int meshVertexId,
    const CurvenetCutResult& curvenetCutResult
)
{
    for (
        size_t i = 0;
        i < curvenetCutResult.sharedCurvenetNodes.size();
        ++i
    )
    {
        if (
            curvenetCutResult
                .sharedCurvenetNodes[i]
                .meshVertexId ==
            meshVertexId
        )
        {
            return static_cast<int>(i);
        }
    }

    return -1;
}

std::string buildControlName(
    const std::string& ownerName,
    int controlId,
    int meshVertexId
)
{
    std::ostringstream name;

    name
        << ownerName
        << "_curvenetControl_"
        << controlId
        << "_v"
        << meshVertexId;

    return name.str();
}

std::string buildCurveName(
    const std::string& ownerName,
    const CurvenetEdge& edge
)
{
    std::ostringstream name;

    name
        << ownerName
        << "_curvenetCurve_"
        << edge.id
        << "_profile_"
        << edge.sourceCurveId;

    return name.str();
}

std::string buildCurveShapeName(
    const std::string& ownerName,
    const CurvenetEdge& edge
)
{
    return buildCurveName(ownerName, edge) + "Shape";
}

void createCurvenetControl(
    const std::string& ownerName,
    int controlId,
    int meshVertexId,
    const Point3& position
)
{
    const std::string controlName =
        buildControlName(
            ownerName,
            controlId,
            meshVertexId
        );

    std::ostringstream command;

    command
        << "string $s[]=`polySphere -r 0.06 -sx 12 -sy 12 -name \""
        << controlName
        << "\"`;";

    command
        << "move "
        << position.x << " "
        << position.y << " "
        << position.z
        << " $s[0];";

    command
        << "parent $s[0] "
        << ownerName
        << "_curvenet_controls;";

    MGlobal::executeCommand(
        command.str().c_str(),
        false,
        false
    );
}

std::vector<int> buildEdgeDisplayVertexIds(
    const CurvenetEdge& edge,
    const CurvenetCutResult& curvenetCutResult
)
{
    std::vector<int> vertexIds;

    if (
        edge.startNode < 0 ||
        edge.startNode >=
            static_cast<int>(
                curvenetCutResult.sharedCurvenetNodes.size()
            ) ||
        edge.endNode < 0 ||
        edge.endNode >=
            static_cast<int>(
                curvenetCutResult.sharedCurvenetNodes.size()
            )
    )
    {
        return vertexIds;
    }

    const int startVertexId =
        curvenetCutResult
            .sharedCurvenetNodes[edge.startNode]
            .meshVertexId;

    const int endVertexId =
        curvenetCutResult
            .sharedCurvenetNodes[edge.endNode]
            .meshVertexId;

    const auto chainIterator =
        curvenetCutResult
            .cutChainsByCurveId
            .find(edge.sourceCurveId);

    if (chainIterator ==
        curvenetCutResult.cutChainsByCurveId.end())
    {
        return vertexIds;
    }

    const CutChain& cutChain =
        chainIterator->second;

    if (cutChain.closed)
    {
        vertexIds = cutChain.vertexIds;

        if (!vertexIds.empty())
        {
            vertexIds.push_back(vertexIds.front());
        }

        return vertexIds;
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
            startVertexId)
        {
            startIndex = vertexIndex;
        }

        if (cutChain.vertexIds[vertexIndex] ==
            endVertexId)
        {
            endIndex = vertexIndex;
        }
    }

    if (startIndex < 0 ||
        endIndex < 0 ||
        cutChain.vertexIds.empty())
    {
        return vertexIds;
    }

    int currentIndex = startIndex;

    while (true)
    {
        vertexIds.push_back(
            cutChain.vertexIds[currentIndex]
        );

        if (currentIndex == endIndex)
        {
            break;
        }

        ++currentIndex;

        if (currentIndex >=
            static_cast<int>(
                cutChain.vertexIds.size()
            ))
        {
            if (!cutChain.closed)
            {
                vertexIds.clear();
                return vertexIds;
            }

            currentIndex = 0;
        }

        if (currentIndex == startIndex)
        {
            vertexIds.clear();
            return vertexIds;
        }
    }

    return vertexIds;
}

std::vector<Point3> buildEdgeDisplayPoints(
    const CurvenetEdge& edge,
    const CurvenetCutResult& curvenetCutResult
)
{
    if (edge.sampledPoints.size() >= 2)
    {
        return edge.sampledPoints;
    }

    const std::vector<int> vertexIds =
        buildEdgeDisplayVertexIds(
            edge,
            curvenetCutResult
        );

    std::vector<Point3> points;

    for (int meshVertexId : vertexIds)
    {
        if (
            meshVertexId >= 0 &&
            meshVertexId <
                static_cast<int>(
                    curvenetCutResult.mesh.vertices.size()
                )
        )
        {
            points.push_back(
                curvenetCutResult
                    .mesh
                    .vertices[meshVertexId]
                    .position
            );
        }
    }

    return points.size() >= 2
        ? points
        : edge.sampledPoints;
}

void createCurvenetCurve(
    const std::string& ownerName,
    const CurvenetEdge& edge,
    const CurvenetCutResult& curvenetCutResult
)
{
    const std::vector<Point3> displayPoints =
        buildEdgeDisplayPoints(
            edge,
            curvenetCutResult
        );

    if (displayPoints.size() < 2)
    {
        return;
    }

    const std::string curveName =
        buildCurveName(
            ownerName,
            edge
        );

    std::ostringstream command;

    command
        << "string $curve=`curve -d 1";

    for (const Point3& point :
         displayPoints)
    {
        command
            << " -p "
            << point.x << " "
            << point.y << " "
            << point.z;
    }

    command
        << " -name \""
        << curveName
        << "\"`;";

    command
        << "string $shapes[]=`listRelatives -shapes $curve`;";

    command
        << "if (size($shapes) > 0) rename $shapes[0] \""
        << buildCurveShapeName(
            ownerName,
            edge
        )
        << "\";";

    command
        << "parent "
        << "$curve"
        << " "
        << ownerName
        << "_curvenet_curves;";

    MGlobal::executeCommand(
        command.str().c_str(),
        false,
        false
    );

}

void createSharedNodeExpression(
    const std::string& ownerName,
    const CurvenetCutResult& curvenetCutResult
)
{
    std::vector<std::vector<std::string>>
        componentsByNode(
            curvenetCutResult.sharedCurvenetNodes.size()
        );

    for (const CurvenetEdge& edge :
         curvenetCutResult.curvenetEdges)
    {
        const std::vector<
            std::pair<
                std::string,
                std::vector<int>
            >
        > curveLayers =
        {
            {
                buildCurveShapeName(
                    ownerName,
                    edge
                ),
                buildEdgeDisplayVertexIds(
                    edge,
                    curvenetCutResult
                )
            }
        };

        for (const auto& layer : curveLayers)
        {
            const std::string& shapeName =
                layer.first;

            const std::vector<int>& vertexIds =
                layer.second;

            for (int cvIndex = 0;
                 cvIndex <
                     static_cast<int>(
                         vertexIds.size()
                     );
                 ++cvIndex)
            {
                const int nodeId =
                    findSharedNodeIndex(
                        vertexIds[cvIndex],
                        curvenetCutResult
                    );

                if (nodeId < 0)
                {
                    continue;
                }

                std::ostringstream component;

                component
                    << shapeName
                    << ".controlPoints["
                    << cvIndex
                    << "]";

                componentsByNode[nodeId]
                    .push_back(component.str());
            }
        }
    }

    std::ostringstream expression;

    for (int nodeId = 0;
         nodeId <
             static_cast<int>(
                 curvenetCutResult.sharedCurvenetNodes.size()
             );
         ++nodeId)
    {
        const int meshVertexId =
            curvenetCutResult
                .sharedCurvenetNodes[nodeId]
                .meshVertexId;

        const std::string controlName =
            buildControlName(
                ownerName,
                nodeId,
                meshVertexId
            );

        for (const std::string& component :
             componentsByNode[nodeId])
        {
            expression
                << component
                << ".xValue = "
                << controlName
                << ".translateX;\n";

            expression
                << component
                << ".yValue = "
                << controlName
                << ".translateY;\n";

            expression
                << component
                << ".zValue = "
                << controlName
                << ".translateZ;\n";
        }

    }

    if (expression.str().empty())
    {
        return;
    }

    std::string expressionText =
        expression.str();

    std::string escapedExpression;

    escapedExpression.reserve(
        expressionText.size()
    );

    for (char character : expressionText)
    {
        if (character == '"')
        {
            escapedExpression += "\\\"";
        }
        else if (character == '\n')
        {
            escapedExpression += "\\n";
        }
        else
        {
            escapedExpression += character;
        }
    }

    std::ostringstream command;

    command
        << "expression -n \""
        << ownerName
        << "_curvenetSharedNodeExpression\" "
        << "-ae true "
        << "-uc all "
        << "-s \""
        << escapedExpression
        << "\";";

    MGlobal::executeCommand(
        command.str().c_str(),
        false,
        false
    );

}

}

void CurvenetSceneBuilder::build(
    const CurvenetCutResult& curvenetCutResult,
    const MString& ownerName
)
{
    const std::string owner =
        ownerName.asChar();

    MGlobal::executeCommand(
        (
            "if (`objExists " + owner +
            "_curvenet_group`) delete " + owner +
            "_curvenet_group;"
        ).c_str(),
        false,
        false
    );

    MGlobal::executeCommand(
        (
            "group -em -name " + owner +
            "_curvenet_group;"
        ).c_str(),
        false,
        false
    );

    MGlobal::executeCommand(
        (
            "group -em -name " + owner +
            "_curvenet_controls -parent " + owner +
            "_curvenet_group;"
        ).c_str(),
        false,
        false
    );

    MGlobal::executeCommand(
        (
            "group -em -name " + owner +
            "_curvenet_curves -parent " + owner +
            "_curvenet_group;"
        ).c_str(),
        false,
        false
    );

    for (
        size_t i = 0;
        i < curvenetCutResult.sharedCurvenetNodes.size();
        ++i
    )
    {
        const SharedCurvenetNode& sharedNode =
            curvenetCutResult.sharedCurvenetNodes[i];

        if (
            sharedNode.meshVertexId < 0 ||
            sharedNode.meshVertexId >=
                static_cast<int>(
                    curvenetCutResult.mesh.vertices.size()
                )
        )
        {
            continue;
        }

        const Point3& position =
            curvenetCutResult
                .mesh
                .vertices[sharedNode.meshVertexId]
                .position;

        createCurvenetControl(
            owner,
            static_cast<int>(i),
            sharedNode.meshVertexId,
            position
        );
    }

    for (
        const CurvenetEdge& edge :
        curvenetCutResult.curvenetEdges
    )
    {
        if (
            edge.startNode < 0 ||
            edge.endNode < 0
        )
        {
            continue;
        }

        createCurvenetCurve(
            owner,
            edge,
            curvenetCutResult
        );
    }

    CurvenetRegionPreviewBuilder::build(
        owner,
        curvenetCutResult
    );

    createSharedNodeExpression(
        owner,
        curvenetCutResult
    );
}
