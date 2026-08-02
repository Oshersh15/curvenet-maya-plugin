#include "CurvenetFaceBuilder.h"

#include <unordered_map>
#include <unordered_set>

namespace
{
struct BoundarySection
{
    int curveId = -1;
    int startVertexId = -1;
    int endVertexId = -1;
};
}

std::vector<CurvenetFace>
CurvenetFaceBuilder::build(
    const CurvenetCutResult& cutResult
)
{
    std::vector<CurvenetFace> faces;

    std::unordered_set<int> sharedVertexIds;

    for (const SharedCurvenetNode& sharedNode :
         cutResult.sharedCurvenetNodes)
    {
        if (sharedNode.meshVertexId >= 0)
        {
            sharedVertexIds.insert(
                sharedNode.meshVertexId
            );
        }
    }

    std::vector<BoundarySection> sections;

    /*
        Divide each CutChain into sections between
        consecutive shared Curvenet nodes.
    */
    for (const auto& entry :
         cutResult.cutChainsByCurveId)
    {
        const int curveId =
            entry.first;

        const CutChain& cutChain =
            entry.second;

        std::vector<int> sharedIndices;

        for (int vertexIndex = 0;
             vertexIndex <
                 static_cast<int>(
                     cutChain.vertexIds.size()
                 );
             ++vertexIndex)
        {
            const int meshVertexId =
                cutChain.vertexIds[
                    vertexIndex
                ];

            if (
                sharedVertexIds.find(
                    meshVertexId
                ) != sharedVertexIds.end()
            )
            {
                sharedIndices.push_back(
                    vertexIndex
                );
            }
        }

        for (int sharedIndex = 0;
             sharedIndex + 1 <
                 static_cast<int>(
                     sharedIndices.size()
                 );
             ++sharedIndex)
        {
            BoundarySection section;

            section.curveId =
                curveId;

            section.startVertexId =
                cutChain.vertexIds[
                    sharedIndices[sharedIndex]
                ];

            section.endVertexId =
                cutChain.vertexIds[
                    sharedIndices[sharedIndex + 1]
                ];

            if (section.startVertexId !=
                section.endVertexId)
            {
                sections.push_back(
                    section
                );
            }
        }

        /*
            A closed CutChain also has a section
            from its final shared node back to its first.
        */
        if (cutChain.closed &&
            sharedIndices.size() >= 2)
        {
            BoundarySection closingSection;

            closingSection.curveId =
                curveId;

            closingSection.startVertexId =
                cutChain.vertexIds[
                    sharedIndices.back()
                ];

            closingSection.endVertexId =
                cutChain.vertexIds[
                    sharedIndices.front()
                ];

            if (closingSection.startVertexId !=
                closingSection.endVertexId)
            {
                sections.push_back(
                    closingSection
                );
            }
        }
    }

    std::unordered_map<int, std::vector<int>>
        sectionIdsByVertex;

    for (int sectionId = 0;
         sectionId <
             static_cast<int>(
                 sections.size()
             );
         ++sectionId)
    {
        const BoundarySection& section =
            sections[sectionId];

        sectionIdsByVertex[
            section.startVertexId
        ].push_back(sectionId);

        sectionIdsByVertex[
            section.endVertexId
        ].push_back(sectionId);
    }

    std::unordered_set<int> visitedSectionIds;

    /*
        A connected boundary component whose nodes
        all have degree two forms one simple loop.
    */
    for (int startingSectionId = 0;
         startingSectionId <
             static_cast<int>(
                 sections.size()
             );
         ++startingSectionId)
    {
        if (
            visitedSectionIds.find(
                startingSectionId
            ) != visitedSectionIds.end()
        )
        {
            continue;
        }

        const BoundarySection& startingSection =
            sections[startingSectionId];

        const int startingVertexId =
            startingSection.startVertexId;

        int currentVertexId =
            startingVertexId;

        int currentSectionId =
            startingSectionId;

        CurvenetFace face;
        face.id =
            static_cast<int>(
                faces.size()
            );

        bool validLoop = true;

        while (true)
        {
            if (
                visitedSectionIds.find(
                    currentSectionId
                ) != visitedSectionIds.end()
            )
            {
                validLoop =
                    currentVertexId ==
                    startingVertexId;

                break;
            }

            const BoundarySection& section =
                sections[currentSectionId];

            CurvenetFaceBoundary boundary;

            boundary.curveId =
                section.curveId;

            if (currentVertexId ==
                section.startVertexId)
            {
                boundary.startVertexId =
                    section.startVertexId;

                boundary.endVertexId =
                    section.endVertexId;

                boundary.reversed = false;
            }
            else if (currentVertexId ==
                     section.endVertexId)
            {
                boundary.startVertexId =
                    section.endVertexId;

                boundary.endVertexId =
                    section.startVertexId;

                boundary.reversed = true;
            }
            else
            {
                validLoop = false;
                break;
            }

            face.boundary.push_back(
                boundary
            );

            visitedSectionIds.insert(
                currentSectionId
            );

            currentVertexId =
                boundary.endVertexId;

            if (currentVertexId ==
                startingVertexId)
            {
                break;
            }

            const auto adjacencyIterator =
                sectionIdsByVertex.find(
                    currentVertexId
                );

            if (adjacencyIterator ==
                    sectionIdsByVertex.end() ||
                adjacencyIterator->second.size() != 2)
            {
                validLoop = false;
                break;
            }

            const std::vector<int>& adjacentSections =
                adjacencyIterator->second;

            currentSectionId =
                adjacentSections[0] ==
                    currentSectionId
                    ? adjacentSections[1]
                    : adjacentSections[0];
        }

        if (validLoop &&
            face.boundary.size() >= 3 &&
            currentVertexId ==
                startingVertexId)
        {
            faces.push_back(
                face
            );
        }
    }

    return faces;
}
