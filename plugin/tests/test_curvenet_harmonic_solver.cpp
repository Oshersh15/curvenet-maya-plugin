#include "CurvenetHarmonicSolver.h"

#include <gtest/gtest.h>

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
