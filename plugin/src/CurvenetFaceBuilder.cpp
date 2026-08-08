#include "CurvenetFaceBuilder.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <set>

namespace
{
    struct BoundarySection
    {
        int curveId = -1;

        int startVertexId = -1;
        int endVertexId = -1;

        /*
            The first CutChain vertex after the start node
            and the final CutChain vertex before the end node.

            These identify the local direction in which the
            section leaves each shared Curvenet node.
        */
        int startNeighbourVertexId = -1;
        int endNeighbourVertexId = -1;
    };

    struct DirectedBoundarySection
    {
        int sectionId = -1;

        bool reversed = false;

        bool visited = false;
    };

    struct GraphBoundaryEdge
    {
        int curveId = -1;
        int startNodeId = -1;
        int endNodeId = -1;
        int startVertexId = -1;
        int endVertexId = -1;
    };

    struct GraphCycle
    {
        std::vector<int> nodeLoop;
        std::vector<int> edgeLoop;
    };

    std::vector<int> buildCanonicalSectionLoop(
        const std::vector<int>& sectionIds
    )
    {
        if (sectionIds.empty())
        {
            return {};
        }

        const auto findSmallestRotation =
            [](
                const std::vector<int>& values
            )
            {
                std::vector<int> smallest =
                    values;

                for (int offset = 1;
                     offset <
                         static_cast<int>(
                             values.size()
                         );
                     ++offset)
                {
                    std::vector<int> rotated;

                    rotated.reserve(
                        values.size()
                    );

                    for (int index = 0;
                         index <
                             static_cast<int>(
                                 values.size()
                             );
                         ++index)
                    {
                        rotated.push_back(
                            values[
                                (
                                    offset + index
                                ) %
                                values.size()
                            ]
                        );
                    }

                    if (rotated < smallest)
                    {
                        smallest =
                            rotated;
                    }
                }

                return smallest;
            };

        const std::vector<int> forward =
            findSmallestRotation(
                sectionIds
            );

        std::vector<int> reversed =
            sectionIds;

        std::reverse(
            reversed.begin(),
            reversed.end()
        );

        reversed =
            findSmallestRotation(
                reversed
            );

        return reversed < forward
            ? reversed
            : forward;
    }

    std::vector<CurvenetFace>
    buildLegacyFaces(
        const std::vector<BoundarySection>& sections
    )
    {
        std::vector<CurvenetFace> faces;

        std::vector<
            std::vector<int>
        > acceptedSectionLoops;

        std::unordered_map<
            int,
            std::vector<int>
        > sectionIdsByVertex;

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
            ].push_back(
                sectionId
            );

            sectionIdsByVertex[
                section.endVertexId
            ].push_back(
                sectionId
            );
        }

        std::unordered_set<int>
            visitedSectionIds;

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

                if (
                    adjacencyIterator ==
                        sectionIdsByVertex.end() ||
                    adjacencyIterator
                        ->second
                        .size() != 2
                )
                {
                    validLoop = false;
                    break;
                }

                const std::vector<int>&
                    adjacentSections =
                        adjacencyIterator->second;

                currentSectionId =
                    adjacentSections[0] ==
                        currentSectionId
                        ? adjacentSections[1]
                        : adjacentSections[0];
            }

            if (
                validLoop &&
                face.boundary.size() >= 3 &&
                currentVertexId ==
                    startingVertexId
            )
            {
                faces.push_back(
                    face
                );
            }
        }

        return faces;
    }

    int getDirectedStartVertexId(
        const BoundarySection& section,
        bool reversed
    )
    {
        return reversed
            ? section.endVertexId
            : section.startVertexId;
    }

    int getDirectedEndVertexId(
        const BoundarySection& section,
        bool reversed
    )
    {
        return reversed
            ? section.startVertexId
            : section.endVertexId;
    }

    int getDirectedStartNeighbourVertexId(
        const BoundarySection& section,
        bool reversed
    )
    {
        return reversed
            ? section.endNeighbourVertexId
            : section.startNeighbourVertexId;
    }

    std::vector<CurvenetFace>
    buildDirectedFaces(
        const std::vector<BoundarySection>& sections,
        const std::unordered_map<
            int,
            std::vector<DirectedBoundarySection>
        >& orderedOutgoingSectionsByVertex
    )
    {
        std::vector<CurvenetFace> faces;

        std::vector<
            std::vector<int>
        > acceptedSectionLoops;

        std::unordered_set<int>
            visitedDirectedSectionIds;

        const auto makeDirectedId =
            [](
                int sectionId,
                bool reversed
            )
            {
                return sectionId * 2 +
                    (reversed ? 1 : 0);
            };

        for (int sectionId = 0;
             sectionId <
                 static_cast<int>(
                     sections.size()
                 );
             ++sectionId)
        {
            for (int direction = 0;
                 direction < 2;
                 ++direction)
            {
                const bool startingReversed =
                    direction == 1;

                const int startingDirectedId =
                    makeDirectedId(
                        sectionId,
                        startingReversed
                    );

                if (
                    visitedDirectedSectionIds.find(
                        startingDirectedId
                    ) !=
                    visitedDirectedSectionIds.end()
                )
                {
                    continue;
                }

                int currentSectionId =
                    sectionId;

                bool currentReversed =
                    startingReversed;

                CurvenetFace face;

                face.id =
                    static_cast<int>(
                        faces.size()
                    );

                std::vector<int>
                    traversedSectionIds;


                bool validLoop = true;

                while (true)
                {
                    const int currentDirectedId =
                        makeDirectedId(
                            currentSectionId,
                            currentReversed
                        );

                    if (
                        visitedDirectedSectionIds.find(
                            currentDirectedId
                        ) !=
                        visitedDirectedSectionIds.end()
                    )
                    {
                        validLoop =
                            currentDirectedId ==
                            startingDirectedId;

                        break;
                    }

                    visitedDirectedSectionIds.insert(currentDirectedId);

                    const BoundarySection& section =
                        sections[
                            currentSectionId
                        ];

                    traversedSectionIds.push_back(
                        currentSectionId
                    );

                    CurvenetFaceBoundary boundary;

                    boundary.curveId =
                        section.curveId;

                    boundary.startVertexId =
                        getDirectedStartVertexId(
                            section,
                            currentReversed
                        );

                    boundary.endVertexId =
                        getDirectedEndVertexId(
                            section,
                            currentReversed
                        );

                    boundary.reversed =
                        currentReversed;

                    face.boundary.push_back(
                        boundary
                    );

                    const int arrivalVertexId =
                        boundary.endVertexId;

                    const auto outgoingIterator =
                        orderedOutgoingSectionsByVertex.find(
                            arrivalVertexId
                        );

                    if (
                        outgoingIterator ==
                        orderedOutgoingSectionsByVertex.end() ||
                        outgoingIterator->second.empty()
                    )
                    {
                        validLoop = false;
                        break;
                    }

                    const std::vector<
                        DirectedBoundarySection
                    >& outgoingSections =
                        outgoingIterator->second;

                    const int reverseSectionId =
                        currentSectionId;

                    const bool reverseDirection =
                        !currentReversed;

                    int reverseIndex = -1;

                    for (int outgoingIndex = 0;
                         outgoingIndex <
                             static_cast<int>(
                                 outgoingSections.size()
                             );
                         ++outgoingIndex)
                    {
                        const DirectedBoundarySection&
                            outgoingSection =
                                outgoingSections[
                                    outgoingIndex
                                ];

                        if (
                            outgoingSection.sectionId ==
                                reverseSectionId &&
                            outgoingSection.reversed ==
                                reverseDirection
                        )
                        {
                            reverseIndex =
                                outgoingIndex;

                            break;
                        }
                    }

                    if (reverseIndex < 0)
                    {
                        validLoop = false;
                        break;
                    }

                    const int nextIndex =
                        (
                            reverseIndex + 1
                        ) %
                        static_cast<int>(
                            outgoingSections.size()
                        );

                    currentSectionId =
                        outgoingSections[
                            nextIndex
                        ].sectionId;

                    currentReversed =
                        outgoingSections[
                            nextIndex
                        ].reversed;

                    if (
                        currentSectionId ==
                            sectionId &&
                        currentReversed ==
                            startingReversed
                    )
                    {
                        break;
                    }
                }

                if (
                    validLoop &&
                    face.boundary.size() >= 3
                )
                {
                    const std::vector<int>
                        canonicalLoop =
                            buildCanonicalSectionLoop(
                                traversedSectionIds
                            );

                    const bool alreadyStored =
                        std::find(
                            acceptedSectionLoops.begin(),
                            acceptedSectionLoops.end(),
                            canonicalLoop
                        ) !=
                        acceptedSectionLoops.end();

                    if (!alreadyStored)
                    {
                        face.id =
                            static_cast<int>(
                                faces.size()
                            );

                        acceptedSectionLoops.push_back(
                            canonicalLoop
                        );

                        faces.push_back(
                            face
                        );
                    }
                }
            }
        }

        return faces;
    }

    std::vector<int> buildCanonicalNodeLoop(
        const std::vector<int>& nodeIds
    )
    {
        if (nodeIds.empty())
        {
            return {};
        }

        std::vector<int> smallest = nodeIds;

        for (int offset = 1;
             offset < static_cast<int>(nodeIds.size());
             ++offset)
        {
            std::vector<int> rotated;
            rotated.reserve(nodeIds.size());

            for (int index = 0;
                 index < static_cast<int>(nodeIds.size());
                 ++index)
            {
                rotated.push_back(
                    nodeIds[(offset + index) % nodeIds.size()]
                );
            }

            if (rotated < smallest)
            {
                smallest = rotated;
            }
        }

        std::vector<int> reversed = nodeIds;
        std::reverse(reversed.begin(), reversed.end());

        std::vector<int> smallestReversed = reversed;

        for (int offset = 1;
             offset < static_cast<int>(reversed.size());
             ++offset)
        {
            std::vector<int> rotated;
            rotated.reserve(reversed.size());

            for (int index = 0;
                 index < static_cast<int>(reversed.size());
                 ++index)
            {
                rotated.push_back(
                    reversed[(offset + index) % reversed.size()]
                );
            }

            if (rotated < smallestReversed)
            {
                smallestReversed = rotated;
            }
        }

        return smallestReversed < smallest
            ? smallestReversed
            : smallest;
    }

    void findGraphCyclesFromNode(
        int startNodeId,
        int currentNodeId,
        const std::vector<GraphBoundaryEdge>& graphEdges,
        const std::unordered_map<int, std::vector<int>>& edgeIdsByNode,
        std::vector<int>& nodeStack,
        std::vector<int>& edgeStack,
        std::unordered_set<int>& nodesInStack,
        std::set<std::vector<int>>& acceptedNodeLoops,
        std::vector<GraphCycle>& acceptedCycles
    )
    {
        const auto adjacencyIterator =
            edgeIdsByNode.find(currentNodeId);

        if (adjacencyIterator == edgeIdsByNode.end())
        {
            return;
        }

        for (int edgeId : adjacencyIterator->second)
        {
            if (std::find(edgeStack.begin(),
                          edgeStack.end(),
                          edgeId) != edgeStack.end())
            {
                continue;
            }

            const GraphBoundaryEdge& edge =
                graphEdges[edgeId];

            const int nextNodeId =
                edge.startNodeId == currentNodeId
                    ? edge.endNodeId
                    : edge.startNodeId;

            if (nextNodeId == startNodeId)
            {
                if (nodeStack.size() >= 3)
                {
                    const std::vector<int> canonicalLoop =
                        buildCanonicalNodeLoop(nodeStack);

                    if (acceptedNodeLoops.insert(canonicalLoop).second)
                    {
                        GraphCycle cycle;
                        cycle.nodeLoop = nodeStack;
                        cycle.edgeLoop = edgeStack;
                        cycle.edgeLoop.push_back(edgeId);
                        acceptedCycles.push_back(cycle);
                    }
                }

                continue;
            }

            if (nextNodeId < startNodeId ||
                nodesInStack.find(nextNodeId) != nodesInStack.end())
            {
                continue;
            }

            nodeStack.push_back(nextNodeId);
            edgeStack.push_back(edgeId);
            nodesInStack.insert(nextNodeId);

            findGraphCyclesFromNode(
                startNodeId,
                nextNodeId,
                graphEdges,
                edgeIdsByNode,
                nodeStack,
                edgeStack,
                nodesInStack,
                acceptedNodeLoops,
                acceptedCycles
            );

            nodesInStack.erase(nextNodeId);
            edgeStack.pop_back();
            nodeStack.pop_back();
        }
    }

    std::vector<CurvenetFace> buildGraphCycleFaces(
        const CurvenetCutResult& cutResult
    )
    {
        std::unordered_map<int, int> nodeIdByMeshVertexId;

        for (int nodeId = 0;
             nodeId <
                 static_cast<int>(
                     cutResult.sharedCurvenetNodes.size()
                 );
             ++nodeId)
        {
            const int meshVertexId =
                cutResult.sharedCurvenetNodes[nodeId]
                    .meshVertexId;

            if (meshVertexId >= 0)
            {
                nodeIdByMeshVertexId[meshVertexId] =
                    nodeId;
            }
        }

        std::vector<GraphBoundaryEdge> graphEdges;

        for (const auto& entry :
             cutResult.cutChainsByCurveId)
        {
            const CutChain& cutChain = entry.second;

            std::vector<int> sharedNodeIds;

            for (int meshVertexId : cutChain.vertexIds)
            {
                const auto nodeIterator =
                    nodeIdByMeshVertexId.find(meshVertexId);

                if (nodeIterator !=
                    nodeIdByMeshVertexId.end())
                {
                    if (sharedNodeIds.empty() ||
                        sharedNodeIds.back() !=
                            nodeIterator->second)
                    {
                        sharedNodeIds.push_back(
                            nodeIterator->second
                        );
                    }
                }
            }

            if (sharedNodeIds.size() < 2)
            {
                continue;
            }

            GraphBoundaryEdge edge;
            edge.curveId = cutChain.curveId;
            edge.startNodeId = sharedNodeIds.front();
            edge.endNodeId = sharedNodeIds.back();
            edge.startVertexId =
                cutResult.sharedCurvenetNodes[
                    edge.startNodeId
                ].meshVertexId;
            edge.endVertexId =
                cutResult.sharedCurvenetNodes[
                    edge.endNodeId
                ].meshVertexId;

            if (edge.startNodeId != edge.endNodeId)
            {
                graphEdges.push_back(edge);
            }
        }

        std::unordered_map<int, std::vector<int>>
            edgeIdsByNode;

        for (int edgeId = 0;
             edgeId < static_cast<int>(graphEdges.size());
             ++edgeId)
        {
            edgeIdsByNode[
                graphEdges[edgeId].startNodeId
            ].push_back(edgeId);

            edgeIdsByNode[
                graphEdges[edgeId].endNodeId
            ].push_back(edgeId);
        }

        std::set<std::vector<int>> acceptedNodeLoops;
        std::vector<GraphCycle> acceptedCycles;

        for (const auto& entry : edgeIdsByNode)
        {
            const int startNodeId = entry.first;

            std::vector<int> nodeStack{ startNodeId };
            std::vector<int> edgeStack;
            std::unordered_set<int> nodesInStack{ startNodeId };

            findGraphCyclesFromNode(
                startNodeId,
                startNodeId,
                graphEdges,
                edgeIdsByNode,
                nodeStack,
                edgeStack,
                nodesInStack,
                acceptedNodeLoops,
                acceptedCycles
            );
        }

        std::vector<CurvenetFace> faces;

        for (const GraphCycle& cycle :
             acceptedCycles)
        {
            CurvenetFace face;
            face.id = static_cast<int>(faces.size());

            if (cycle.edgeLoop.empty() ||
                cycle.nodeLoop.empty())
            {
                continue;
            }

            int currentNodeId = cycle.nodeLoop.front();

            for (int edgeId : cycle.edgeLoop)
            {
                const GraphBoundaryEdge& edge =
                    graphEdges[edgeId];

                CurvenetFaceBoundary boundary;
                boundary.curveId = edge.curveId;

                if (edge.startNodeId == currentNodeId)
                {
                    boundary.startVertexId =
                        edge.startVertexId;
                    boundary.endVertexId =
                        edge.endVertexId;
                    boundary.reversed = false;
                    currentNodeId = edge.endNodeId;
                }
                else
                {
                    boundary.startVertexId =
                        edge.endVertexId;
                    boundary.endVertexId =
                        edge.startVertexId;
                    boundary.reversed = true;
                    currentNodeId = edge.startNodeId;
                }

                face.boundary.push_back(boundary);
            }

            if (face.boundary.size() >= 3)
            {
                faces.push_back(face);
            }
        }

        return faces;
    }

}

std::vector<CurvenetFace>
CurvenetFaceBuilder::build(
    const CurvenetCutResult& cutResult
)
{

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

            const int startIndex =
                sharedIndices[sharedIndex];

            const int endIndex =
                sharedIndices[sharedIndex + 1];

            if (startIndex + 1 < endIndex)
            {
                section.startNeighbourVertexId =
                    cutChain.vertexIds[
                        startIndex + 1
                    ];

                section.endNeighbourVertexId =
                    cutChain.vertexIds[
                        endIndex - 1
                    ];
            }

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

            const int lastSharedIndex =
                sharedIndices.back();

            const int firstSharedIndex =
                sharedIndices.front();

            const int vertexCount =
                static_cast<int>(
                    cutChain.vertexIds.size()
                );

            if (vertexCount > 2)
            {
                closingSection.startNeighbourVertexId =
                    cutChain.vertexIds[
                        (lastSharedIndex + 1) %
                        vertexCount
                    ];

                closingSection.endNeighbourVertexId =
                    cutChain.vertexIds[
                        (firstSharedIndex - 1 +
                         vertexCount) %
                        vertexCount
                    ];
            }

            if (closingSection.startVertexId !=
                closingSection.endVertexId)
            {
                sections.push_back(
                    closingSection
                );
            }
        }
    }

    std::vector<DirectedBoundarySection>
        directedSections;

    directedSections.reserve(
        sections.size() * 2
    );

    for (int sectionId = 0;
         sectionId <
             static_cast<int>(
                 sections.size()
             );
         ++sectionId)
    {
        DirectedBoundarySection forward;

        forward.sectionId =
            sectionId;

        forward.reversed =
            false;

        directedSections.push_back(
            forward
        );

        DirectedBoundarySection reverse;

        reverse.sectionId =
            sectionId;

        reverse.reversed =
            true;

        directedSections.push_back(
            reverse
        );
    }

    std::unordered_map<
        int,
        std::vector<DirectedBoundarySection>
    > outgoingSectionsByVertex;

    for (const DirectedBoundarySection& directedSection :
         directedSections)
    {
        const BoundarySection& section =
            sections[
                directedSection.sectionId
            ];

        const int startVertexId =
            directedSection.reversed
                ? section.endVertexId
                : section.startVertexId;

        outgoingSectionsByVertex[
            startVertexId
        ].push_back(
            directedSection
        );
    }

    std::unordered_map<
        int,
        std::vector<DirectedBoundarySection>
    > orderedOutgoingSectionsByVertex;

    for (const auto& entry :
         outgoingSectionsByVertex)
    {
        const int sharedVertexId =
            entry.first;

        const std::vector<DirectedBoundarySection>&
            outgoingSections =
                entry.second;

        const std::vector<int>
            orderedOutgoingHalfEdges =
                cutResult.mesh
                    .getOrderedOutgoingHalfEdgesAtVertex(
                        sharedVertexId
                    );

        std::vector<DirectedBoundarySection>&
            orderedSections =
                orderedOutgoingSectionsByVertex[
                    sharedVertexId
                ];

        for (int halfEdgeId :
             orderedOutgoingHalfEdges)
        {
            if (halfEdgeId < 0 ||
                halfEdgeId >=
                    static_cast<int>(
                        cutResult.mesh.halfEdges.size()
                    ))
            {
                continue;
            }

            const int neighbourVertexId =
                cutResult.mesh.halfEdges[
                    halfEdgeId
                ].endVertex;

            for (const DirectedBoundarySection&
                 directedSection :
                 outgoingSections)
            {
                const BoundarySection& section =
                    sections[
                        directedSection.sectionId
                    ];

                const int startNeighbourVertexId =
                    getDirectedStartNeighbourVertexId(
                        section,
                        directedSection.reversed
                    );

                if (startNeighbourVertexId !=
                    neighbourVertexId)
                {
                    continue;
                }

                const bool alreadyAdded =
                    std::find_if(
                        orderedSections.begin(),
                        orderedSections.end(),
                        [&directedSection](
                            const DirectedBoundarySection&
                                existing
                        )
                        {
                            return
                                existing.sectionId ==
                                    directedSection.sectionId &&
                                existing.reversed ==
                                    directedSection.reversed;
                        }
                    ) != orderedSections.end();

                if (!alreadyAdded)
                {
                    orderedSections.push_back(
                        directedSection
                    );
                }
            }
        }
    }

    std::vector<CurvenetFace> directedFaces =
        buildDirectedFaces(
        sections,
        orderedOutgoingSectionsByVertex
    );

    std::vector<CurvenetFace> graphCycleFaces =
        buildGraphCycleFaces(cutResult);

    return graphCycleFaces.size() > directedFaces.size()
        ? graphCycleFaces
        : directedFaces;
}
