/* Tests smooth deformation, rigid motion, reset, and dense topology support. */

#include "CurvenetHarmonicSolver.h"

#include <gtest/gtest.h>

namespace
{
HalfEdgeMesh createLongStrip(int vertexCount)
{
    HalfEdgeMesh mesh;
    mesh.vertices.resize(vertexCount);

    for (int vertexId = 0; vertexId < vertexCount; ++vertexId)
    {
        mesh.vertices[vertexId].position = Point3{
            static_cast<double>(vertexId), 0.0, 0.0
        };

        if (vertexId + 1 < vertexCount)
        {
            mesh.halfEdges.push_back(
                HalfEdge{vertexId, vertexId + 1, -1, -1, -1}
            );
            mesh.halfEdges.push_back(
                HalfEdge{vertexId + 1, vertexId, -1, -1, -1}
            );
        }
    }

    return mesh;
}
}

TEST(CurvenetHarmonicSolver, ReturnsEmptyBeforeInitialization)
{
    CurvenetHarmonicSolver solver;

    EXPECT_FALSE(solver.isInitialized());
    EXPECT_TRUE(solver.solve({}, 30).empty());
}

TEST(CurvenetHarmonicSolver, SpreadsUniformBoundaryTranslationAcrossQuad)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutChain chain;
    chain.curveId = 0;
    chain.points = {
        EmbeddedCurvePoint{0, 0, 0.0, mesh.vertices[0].position},
        EmbeddedCurvePoint{1, 0, 1.0, mesh.vertices[1].position}
    };

    const std::unordered_map<int, CutChain> chains = {{0, chain}};
    const std::vector<std::vector<Point3>> neutral = {{
        mesh.vertices[0].position,
        mesh.vertices[1].position
    }};
    std::vector<std::vector<Point3>> current = neutral;
    current[0][0].x += 2.0;
    current[0][1].x += 2.0;

    CurvenetHarmonicSolver solver;
    solver.initialize(mesh, chains, 4, neutral);
    const std::vector<Point3> result = solver.solve(current, 100);

    ASSERT_EQ(result.size(), 4u);

    for (const Point3& displacement : result)
    {
        EXPECT_NEAR(displacement.x, 2.0, 1.0e-4);
        EXPECT_NEAR(displacement.y, 0.0, 1.0e-6);
        EXPECT_NEAR(displacement.z, 0.0, 1.0e-6);
    }
}

TEST(CurvenetHarmonicSolver, ReturnsExactlyToNeutralAfterADeformedSolve)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutChain chain;
    chain.curveId = 0;
    chain.points = {
        EmbeddedCurvePoint{0, 0, 0.0, mesh.vertices[0].position},
        EmbeddedCurvePoint{1, 0, 1.0, mesh.vertices[1].position}
    };

    const std::unordered_map<int, CutChain> chains = {{0, chain}};
    const std::vector<std::vector<Point3>> neutral = {{
        mesh.vertices[0].position,
        mesh.vertices[1].position
    }};
    std::vector<std::vector<Point3>> deformed = neutral;
    deformed[0][0].y += 1.0;
    deformed[0][1].y += 1.0;

    CurvenetHarmonicSolver solver;
    solver.initialize(mesh, chains, 4, neutral);
    solver.solve(deformed, 100);
    const std::vector<Point3> restored = solver.solve(neutral, 100);

    for (const Point3& displacement : restored)
    {
        EXPECT_DOUBLE_EQ(displacement.x, 0.0);
        EXPECT_DOUBLE_EQ(displacement.y, 0.0);
        EXPECT_DOUBLE_EQ(displacement.z, 0.0);
    }
}

TEST(CurvenetHarmonicSolver, PreservesGlobalRigidRotation)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutChain chain;
    chain.curveId = 0;
    chain.points = {
        EmbeddedCurvePoint{0, 0, 0.0, mesh.vertices[0].position},
        EmbeddedCurvePoint{1, 0, 1.0, mesh.vertices[1].position},
        EmbeddedCurvePoint{2, 1, 1.0, mesh.vertices[2].position}
    };

    const std::unordered_map<int, CutChain> chains = {{0, chain}};
    const std::vector<std::vector<Point3>> neutral = {{
        mesh.vertices[0].position,
        mesh.vertices[1].position,
        mesh.vertices[2].position
    }};
    std::vector<std::vector<Point3>> current = neutral;

    for (Point3& point : current[0])
    {
        const double originalX = point.x;
        point.x = -point.y + 3.0;
        point.y = originalX + 2.0;
    }

    CurvenetHarmonicSolver solver;
    solver.initialize(mesh, chains, 4, neutral);
    const std::vector<Point3> result = solver.solve(current, 100);

    ASSERT_EQ(result.size(), 4u);

    for (int vertexId = 0; vertexId < 4; ++vertexId)
    {
        const Point3& neutralPosition = mesh.vertices[vertexId].position;
        const Point3 expected{
            -neutralPosition.y + 3.0 - neutralPosition.x,
            neutralPosition.x + 2.0 - neutralPosition.y,
            0.0
        };
        EXPECT_NEAR(result[vertexId].x, expected.x, 1.0e-5);
        EXPECT_NEAR(result[vertexId].y, expected.y, 1.0e-5);
        EXPECT_NEAR(result[vertexId].z, expected.z, 1.0e-5);
    }
}

TEST(CurvenetHarmonicSolver, InitializesDenseStripFromBothConstraintSides)
{
    constexpr int vertexCount = 65;
    HalfEdgeMesh mesh = createLongStrip(vertexCount);
    const Point3 left = mesh.vertices.front().position;
    const Point3 right = mesh.vertices.back().position;
    CutChain leftChain;
    leftChain.curveId = 0;
    leftChain.points = {
        EmbeddedCurvePoint{0, 0, 0.0, left}
    };
    CutChain rightChain;
    rightChain.curveId = 1;
    rightChain.points = {
        EmbeddedCurvePoint{vertexCount - 1, 0, 0.0, right}
    };
    const std::unordered_map<int, CutChain> chains = {
        {0, leftChain},
        {1, rightChain}
    };
    const std::vector<std::vector<Point3>> neutral = {
        {left, Point3{left.x, 1.0, left.z}},
        {right, Point3{right.x, 1.0, right.z}}
    };
    std::vector<std::vector<Point3>> current = neutral;
    current[1][0].y += 8.0;
    current[1][1].y += 8.0;

    CurvenetHarmonicSolver solver;
    solver.initialize(mesh, chains, vertexCount, neutral);
    const std::vector<Point3> result = solver.solve(current, 1);

    ASSERT_EQ(result.size(), static_cast<std::size_t>(vertexCount));
    EXPECT_NEAR(result[vertexCount / 2].y, 4.0, 0.25);
}

TEST(CurvenetHarmonicSolver, IgnoresMissingCurrentCurveWithoutMovingMesh)
{
    HalfEdgeMesh mesh;
    mesh.createTestQuad();

    CutChain chain;
    chain.curveId = 0;
    chain.points = {
        EmbeddedCurvePoint{0, 0, 0.0, mesh.vertices[0].position},
        EmbeddedCurvePoint{1, 0, 1.0, mesh.vertices[1].position}
    };
    const std::vector<std::vector<Point3>> neutral = {{
        mesh.vertices[0].position,
        mesh.vertices[1].position
    }};

    CurvenetHarmonicSolver solver;
    solver.initialize(mesh, {{0, chain}}, 4, neutral);
    const std::vector<Point3> result = solver.solve({}, 30);

    ASSERT_EQ(result.size(), 4u);
    for (const Point3& displacement : result)
    {
        EXPECT_DOUBLE_EQ(displacement.x, 0.0);
        EXPECT_DOUBLE_EQ(displacement.y, 0.0);
        EXPECT_DOUBLE_EQ(displacement.z, 0.0);
    }
}
