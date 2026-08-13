#include <gtest/gtest.h>

#include <algorithm>
#include <set>

#include "CurvenetFaceBuilder.h"
#include "CurvenetFaceRegionBuilder.h"
#include "CurvenetMeshCutter.h"

namespace
{
    HalfEdgeMesh createQuadGrid(int width, int height)
    {
        HalfEdgeMesh mesh;

        for (int y = 0; y <= height; ++y)
        {
            for (int x = 0; x <= width; ++x)
            {
                Vertex vertex;
                vertex.position = Point3{
                    static_cast<double>(x),
                    static_cast<double>(y),
                    0.0
                };
                mesh.vertices.push_back(vertex);
            }
        }

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const int bottomLeft = y * (width + 1) + x;
                const int bottomRight = bottomLeft + 1;
                const int topLeft = bottomLeft + width + 1;
                const int topRight = topLeft + 1;
                const int faceId = static_cast<int>(mesh.faces.size());
                const int firstHalfEdgeId =
                    static_cast<int>(mesh.halfEdges.size());

                mesh.halfEdges.push_back(HalfEdge{
                    bottomLeft,
                    bottomRight,
                    firstHalfEdgeId + 1,
                    -1,
                    faceId
                });
                mesh.halfEdges.push_back(HalfEdge{
                    bottomRight,
                    topRight,
                    firstHalfEdgeId + 2,
                    -1,
                    faceId
                });
                mesh.halfEdges.push_back(HalfEdge{
                    topRight,
                    topLeft,
                    firstHalfEdgeId + 3,
                    -1,
                    faceId
                });
                mesh.halfEdges.push_back(HalfEdge{
                    topLeft,
                    bottomLeft,
                    firstHalfEdgeId,
                    -1,
                    faceId
                });

                Face face;
                face.halfEdge = firstHalfEdgeId;
                mesh.faces.push_back(face);
            }
        }

        mesh.assignTwins();
        return mesh;
    }
}

TEST(
    CurvenetMeshCutter,
    TracksProfileThroughDiagonalFacesAtSharedVertices
)
{
    ProfileCutInput profile;
    profile.curveId = 0;

    const std::vector<Point3> samples = {
        Point3{0.5, 0.5, 0.0},
        Point3{0.9, 0.9, 0.0},
        Point3{1.1, 1.1, 0.0},
        Point3{1.9, 1.9, 0.0},
        Point3{2.1, 2.1, 0.0},
        Point3{2.9, 2.9, 0.0},
        Point3{3.1, 3.1, 0.0},
        Point3{3.5, 3.5, 0.0}
    };

    for (int index = 0;
         index + 1 < static_cast<int>(samples.size());
         ++index)
    {
        profile.sampledSegments.push_back(
            PolylineSegment{samples[index], samples[index + 1]}
        );
    }

    const CurvenetCutResult result = CurvenetMeshCutter::apply(
        createQuadGrid(4, 4),
        {profile},
        0.01,
        0.0001
    );

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.surfaceTrackedCurveCount, 1);
    ASSERT_TRUE(result.surfaceTrackingFailures.empty());

    const CutChain& chain = result.cutChainsByCurveId.at(0);
    ASSERT_EQ(chain.vertexIds.size(), 3);
    ASSERT_EQ(chain.halfEdgeIds.size(), 2);

    for (int index = 0; index < 3; ++index)
    {
        const Point3& position =
            result.mesh.vertices[chain.vertexIds[index]].position;
        const double expected = static_cast<double>(index + 1);
        EXPECT_DOUBLE_EQ(position.x, expected);
        EXPECT_DOUBLE_EQ(position.y, expected);
        EXPECT_DOUBLE_EQ(position.z, 0.0);
    }
}

TEST(
    CurvenetMeshCutter,
    EmbedsSampledProfilesMeetingAtExistingMeshVertex
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    ProfileCutInput firstProfile;
    firstProfile.curveId = 0;
    firstProfile.sampledSegments = {
        PolylineSegment{
            Point3{0.0, 0.5, 0.0},
            Point3{1.0, 1.0, 0.0}
        }
    };

    ProfileCutInput secondProfile;
    secondProfile.curveId = 1;
    secondProfile.sampledSegments = {
        PolylineSegment{
            Point3{1.0, 1.0, 0.0},
            Point3{2.0, 0.5, 0.0}
        }
    };

    ProfileCurveConnection connection;
    connection.endpoint = CurveEndpoint::End;
    connection.targetCurveId = 1;
    connection.targetSegmentId = 0;
    connection.targetSegmentT = 0.0;
    firstProfile.connections.push_back(connection);

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {firstProfile, secondProfile},
            0.0001,
            0.0001
        );

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.sharedCurvenetNodes.size(), 1);

    const int sharedVertexId =
        result.sharedCurvenetNodes.front().meshVertexId;

    EXPECT_NE(
        std::find(
            result.cutChainsByCurveId.at(0).vertexIds.begin(),
            result.cutChainsByCurveId.at(0).vertexIds.end(),
            sharedVertexId
        ),
        result.cutChainsByCurveId.at(0).vertexIds.end()
    );
    EXPECT_NE(
        std::find(
            result.cutChainsByCurveId.at(1).vertexIds.begin(),
            result.cutChainsByCurveId.at(1).vertexIds.end(),
            sharedVertexId
        ),
        result.cutChainsByCurveId.at(1).vertexIds.end()
    );
}

TEST(
    CurvenetMeshCutter,
    ConnectsInteriorAuthoredJunctionToPhysicalHalfEdges
)
{
    HalfEdgeMesh inputMesh;
    inputMesh.createFourQuadGrid();

    const Point3 junction{0.75, 0.75, 0.0};

    ProfileCutInput firstProfile;
    firstProfile.curveId = 0;
    firstProfile.sampledSegments = {
        PolylineSegment{
            Point3{2.0, 0.5, 0.0},
            junction
        }
    };

    ProfileCutInput secondProfile;
    secondProfile.curveId = 1;
    secondProfile.sampledSegments = {
        PolylineSegment{
            junction,
            Point3{0.5, 2.0, 0.0}
        }
    };

    ProfileCurveConnection connection;
    connection.endpoint = CurveEndpoint::End;
    connection.targetCurveId = 1;
    connection.targetSegmentId = 0;
    connection.targetSegmentT = 0.0;
    firstProfile.connections.push_back(connection);

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {firstProfile, secondProfile},
            0.0001,
            0.0001
        );

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.sharedCurvenetNodes.size(), 1);

    const int sharedVertexId =
        result.sharedCurvenetNodes.front().meshVertexId;

    EXPECT_FALSE(
        result.mesh
            .getOutgoingHalfEdgesAtVertex(sharedVertexId)
            .empty()
    );
}

TEST(
    CurvenetMeshCutter,
    ConnectsThreeProfilesAtOneInteriorAuthoredJunction
)
{
    HalfEdgeMesh inputMesh = createQuadGrid(4, 4);

    const Point3 junction{2.5, 2.5, 0.0};

    ProfileCutInput firstProfile;
    firstProfile.curveId = 0;
    firstProfile.sampledSegments = {
        PolylineSegment{Point3{0.0, 2.5, 0.0}, junction}
    };

    ProfileCutInput secondProfile;
    secondProfile.curveId = 1;
    secondProfile.sampledSegments = {
        PolylineSegment{junction, Point3{4.0, 2.5, 0.0}}
    };

    ProfileCutInput thirdProfile;
    thirdProfile.curveId = 2;
    thirdProfile.sampledSegments = {
        PolylineSegment{junction, Point3{2.5, 4.0, 0.0}}
    };

    ProfileCurveConnection firstConnection;
    firstConnection.endpoint = CurveEndpoint::End;
    firstConnection.targetCurveId = 1;
    firstConnection.targetSegmentId = 0;
    firstConnection.targetSegmentT = 0.0;
    firstProfile.connections.push_back(firstConnection);

    ProfileCurveConnection thirdConnection;
    thirdConnection.endpoint = CurveEndpoint::Start;
    thirdConnection.targetCurveId = 1;
    thirdConnection.targetSegmentId = 0;
    thirdConnection.targetSegmentT = 0.0;
    thirdProfile.connections.push_back(thirdConnection);

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            {firstProfile, secondProfile, thirdProfile},
            0.0001,
            0.0001
        );

    ASSERT_TRUE(result.success);

    int sharedVertexId = -1;

    for (const SharedCurvenetNode& node :
         result.sharedCurvenetNodes)
    {
        if (node.connectedCurveIds.size() == 3)
        {
            ASSERT_EQ(sharedVertexId, -1);
            sharedVertexId = node.meshVertexId;
        }
    }

    ASSERT_GE(sharedVertexId, 0);

    std::vector<int> junctionNeighbourIds;

    for (int curveId = 0; curveId < 3; ++curveId)
    {
        const CutChain& chain =
            result.cutChainsByCurveId.at(curveId);
        const auto sharedIterator = std::find(
            chain.vertexIds.begin(),
            chain.vertexIds.end(),
            sharedVertexId
        );

        ASSERT_NE(sharedIterator, chain.vertexIds.end());
        const int sharedIndex = static_cast<int>(
            sharedIterator - chain.vertexIds.begin()
        );

        if (sharedIndex == 0 && chain.vertexIds.size() > 1)
        {
            junctionNeighbourIds.push_back(chain.vertexIds[1]);
        }
        else if (sharedIndex > 0)
        {
            junctionNeighbourIds.push_back(
                chain.vertexIds[sharedIndex - 1]
            );
        }
    }

    ASSERT_EQ(junctionNeighbourIds.size(), 3);
    EXPECT_NE(junctionNeighbourIds[0], junctionNeighbourIds[1]);
    EXPECT_NE(junctionNeighbourIds[0], junctionNeighbourIds[2]);
    EXPECT_NE(junctionNeighbourIds[1], junctionNeighbourIds[2])
        << " neighbours by curve: "
        << junctionNeighbourIds[0] << ", "
        << junctionNeighbourIds[1] << ", "
        << junctionNeighbourIds[2];

    EXPECT_EQ(
        result.mesh
            .getOutgoingHalfEdgesAtVertex(sharedVertexId)
            .size(),
        3
    );

    for (int curveId = 0; curveId < 3; ++curveId)
    {
        const CutChain& chain =
            result.cutChainsByCurveId.at(curveId);

        EXPECT_NE(
            std::find(
                chain.vertexIds.begin(),
                chain.vertexIds.end(),
                sharedVertexId
            ),
            chain.vertexIds.end()
        );
    }
}

TEST(
    CurvenetMeshCutter,
    ConnectsFourProfilesAtOneInteriorAuthoredJunction
)
{
    HalfEdgeMesh inputMesh = createQuadGrid(4, 4);

    const Point3 junction{2.5, 2.5, 0.0};

    std::vector<ProfileCutInput> profiles(4);
    profiles[0].curveId = 0;
    profiles[0].sampledSegments = {
        PolylineSegment{Point3{0.0, 2.5, 0.0}, junction}
    };
    profiles[1].curveId = 1;
    profiles[1].sampledSegments = {
        PolylineSegment{junction, Point3{4.0, 2.5, 0.0}}
    };
    profiles[2].curveId = 2;
    profiles[2].sampledSegments = {
        PolylineSegment{Point3{2.5, 0.0, 0.0}, junction}
    };
    profiles[3].curveId = 3;
    profiles[3].sampledSegments = {
        PolylineSegment{junction, Point3{2.5, 4.0, 0.0}}
    };

    for (int curveId : {0, 2, 3})
    {
        ProfileCurveConnection connection;
        connection.endpoint =
            curveId == 3
                ? CurveEndpoint::Start
                : CurveEndpoint::End;
        connection.targetCurveId = 1;
        connection.targetSegmentId = 0;
        connection.targetSegmentT = 0.0;
        profiles[curveId].connections.push_back(connection);
    }

    const CurvenetCutResult result =
        CurvenetMeshCutter::apply(
            inputMesh,
            profiles,
            0.0001,
            0.0001
        );

    ASSERT_TRUE(result.success);

    int sharedVertexId = -1;
    for (const SharedCurvenetNode& node :
         result.sharedCurvenetNodes)
    {
        if (node.connectedCurveIds.size() == 4)
        {
            sharedVertexId = node.meshVertexId;
            break;
        }
    }

    ASSERT_GE(sharedVertexId, 0);
    const std::vector<int> junctionHalfEdgeIds =
        result.mesh.getOutgoingHalfEdgesAtVertex(sharedVertexId);

    ASSERT_EQ(junctionHalfEdgeIds.size(), 4);

    for (int halfEdgeId : junctionHalfEdgeIds)
    {
        EXPECT_TRUE(
            result.embeddedHalfEdgeIds.count(halfEdgeId)
        );
    }

    for (int faceId = 0;
         faceId < static_cast<int>(result.mesh.faces.size());
         ++faceId)
    {
        EXPECT_GE(result.mesh.traverseFace(faceId).size(), 3);
    }

    for (int curveId = 0; curveId < 4; ++curveId)
    {
        const std::vector<int>& vertexIds =
            result.cutChainsByCurveId.at(curveId).vertexIds;
        EXPECT_NE(
            std::find(
                vertexIds.begin(),
                vertexIds.end(),
                sharedVertexId
            ),
            vertexIds.end()
        );
    }
}

TEST(
    CurvenetMeshCutter,
    PreservesLogicalJunctionWhenCutEndpointsDoNotShareAFace
)
{
    HalfEdgeMesh inputMesh = createQuadGrid(4, 4);

    ProfileCutInput lowerProfile;
    lowerProfile.curveId = 0;
    lowerProfile.sampledSegments = {
        PolylineSegment{
            Point3{0.0, 0.5, 0.0},
            Point3{4.0, 0.5, 0.0}
        }
    };

    ProfileCutInput upperProfile;
    upperProfile.curveId = 1;
    upperProfile.sampledSegments = {
        PolylineSegment{
            Point3{0.0, 3.5, 0.0},
            Point3{4.0, 3.5, 0.0}
        }
    };

    ProfileCurveConnection authoredConnection;
    authoredConnection.endpoint = CurveEndpoint::Start;
    authoredConnection.targetCurveId = 1;
    authoredConnection.targetSegmentId = 0;
    authoredConnection.targetSegmentT = 0.0;
    lowerProfile.connections.push_back(authoredConnection);

    const CurvenetCutResult result = CurvenetMeshCutter::apply(
        inputMesh,
        {lowerProfile, upperProfile},
        0.0001,
        0.0001
    );

    ASSERT_TRUE(result.success);

    const auto logicalNode = std::find_if(
        result.sharedCurvenetNodes.begin(),
        result.sharedCurvenetNodes.end(),
        [](const SharedCurvenetNode& node)
        {
            return node.meshVertexId < 0 &&
                node.connectedCurveIds.size() == 2;
        }
    );

    ASSERT_NE(logicalNode, result.sharedCurvenetNodes.end());
    const std::set<int> connectedCurveIds(
        logicalNode->connectedCurveIds.begin(),
        logicalNode->connectedCurveIds.end()
    );
    EXPECT_EQ(connectedCurveIds, (std::set<int>{0, 1}));
    EXPECT_DOUBLE_EQ(logicalNode->position.x, 0.0);
}

TEST(
    CurvenetMeshCutter,
    BuildsThreeFacesForThreeCellAuthoredGrid
)
{
    const std::vector<Point3> nodes = {
        Point3{0.5, 4.5, 0.0},
        Point3{2.5, 4.5, 0.0},
        Point3{4.5, 4.5, 0.0},
        Point3{6.5, 4.5, 0.0},
        Point3{0.5, 2.5, 0.0},
        Point3{2.5, 2.5, 0.0},
        Point3{4.5, 2.5, 0.0},
        Point3{6.5, 2.5, 0.0}
    };

    const std::vector<std::pair<int, int>> edgeNodes = {
        {0, 1}, {1, 2}, {2, 3},
        {4, 5}, {5, 6}, {6, 7},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    std::vector<ProfileCutInput> profiles(edgeNodes.size());

    for (int curveId = 0;
         curveId < static_cast<int>(edgeNodes.size());
         ++curveId)
    {
        profiles[curveId].curveId = curveId;
        profiles[curveId].sampledSegments = {
            PolylineSegment{
                nodes[edgeNodes[curveId].first],
                nodes[edgeNodes[curveId].second]
            }
        };
    }

    const auto connect =
        [&profiles](
            int sourceCurveId,
            CurveEndpoint sourceEndpoint,
            int targetCurveId,
            CurveEndpoint targetEndpoint
        )
        {
            ProfileCurveConnection connection;
            connection.endpoint = sourceEndpoint;
            connection.targetCurveId = targetCurveId;
            connection.targetSegmentId = 0;
            connection.targetSegmentT =
                targetEndpoint == CurveEndpoint::Start
                    ? 0.0
                    : 1.0;
            profiles[sourceCurveId].connections.push_back(connection);
        };

    connect(6, CurveEndpoint::Start, 0, CurveEndpoint::Start);
    connect(1, CurveEndpoint::Start, 0, CurveEndpoint::End);
    connect(7, CurveEndpoint::Start, 0, CurveEndpoint::End);
    connect(2, CurveEndpoint::Start, 1, CurveEndpoint::End);
    connect(8, CurveEndpoint::Start, 1, CurveEndpoint::End);
    connect(9, CurveEndpoint::Start, 2, CurveEndpoint::End);
    connect(6, CurveEndpoint::End, 3, CurveEndpoint::Start);
    connect(4, CurveEndpoint::Start, 3, CurveEndpoint::End);
    connect(7, CurveEndpoint::End, 3, CurveEndpoint::End);
    connect(5, CurveEndpoint::Start, 4, CurveEndpoint::End);
    connect(8, CurveEndpoint::End, 4, CurveEndpoint::End);
    connect(9, CurveEndpoint::End, 5, CurveEndpoint::End);

    CurvenetCutResult result = CurvenetMeshCutter::apply(
        createQuadGrid(8, 8),
        profiles,
        0.0001,
        0.0001
    );

    ASSERT_TRUE(result.success)
        << " curve " << result.failedCurveId
        << " reason "
        << static_cast<int>(result.failedSplitReason)
        << " interval " << result.failedIntervalIndex
        << " vertices " << result.failedFirstVertexId
        << " -> " << result.failedSecondVertexId;
    ASSERT_EQ(result.sharedCurvenetNodes.size(), 8);

    result.curvenetFaces = CurvenetFaceBuilder::build(result);

    ASSERT_EQ(result.curvenetFaces.size(), 4);

    CurvenetFaceRegionBuilder::build(result);

    const auto exteriorIterator = std::max_element(
        result.curvenetFaces.begin(),
        result.curvenetFaces.end(),
        [](const CurvenetFace& first, const CurvenetFace& second)
        {
            return first.meshFaceIds.size() < second.meshFaceIds.size();
        }
    );
    ASSERT_NE(exteriorIterator, result.curvenetFaces.end());
    result.curvenetFaces.erase(exteriorIterator);
    ASSERT_EQ(result.curvenetFaces.size(), 3);

    std::set<int> mappedFaceIds;
    std::vector<std::size_t> mappedCounts;

    for (const CurvenetFace& face : result.curvenetFaces)
    {
        EXPECT_FALSE(face.meshFaceIds.empty());
        mappedCounts.push_back(face.meshFaceIds.size());

        for (int meshFaceId : face.meshFaceIds)
        {
            EXPECT_TRUE(mappedFaceIds.insert(meshFaceId).second)
                << "mesh face " << meshFaceId
                << " belongs to multiple Curvenet faces";
        }
    }

    ASSERT_EQ(mappedCounts.size(), 3);
    EXPECT_EQ(mappedCounts[0], mappedCounts[1]);
    EXPECT_EQ(mappedCounts[1], mappedCounts[2]);
    EXPECT_LT(mappedFaceIds.size(), result.mesh.faces.size() / 2);
}
