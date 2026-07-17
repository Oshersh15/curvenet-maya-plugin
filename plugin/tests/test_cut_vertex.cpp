#include "CutVertex.h"

#include <gtest/gtest.h>

TEST(CutVertex, StoresCutVertexData)
{
    CutVertex cutVertex;

    cutVertex.position =
        Point3{1.0, 2.0, 3.0};

    cutVertex.sourceHalfEdgeId = 12;
    cutVertex.sourceEdgeT = 0.25;
    cutVertex.curveId = 4;
    cutVertex.cutPathOrder = 2;

    EXPECT_DOUBLE_EQ(
        cutVertex.position.x,
        1.0
    );

    EXPECT_DOUBLE_EQ(
        cutVertex.position.y,
        2.0
    );

    EXPECT_DOUBLE_EQ(
        cutVertex.position.z,
        3.0
    );

    EXPECT_EQ(
        cutVertex.sourceHalfEdgeId,
        12
    );

    EXPECT_DOUBLE_EQ(
        cutVertex.sourceEdgeT,
        0.25
    );

    EXPECT_EQ(
        cutVertex.curveId,
        4
    );

    EXPECT_EQ(
        cutVertex.cutPathOrder,
        2
    );
}

TEST(CutVertex, UsesInvalidDefaultIdentifiers)
{
    const CutVertex cutVertex;

    EXPECT_EQ(
        cutVertex.sourceHalfEdgeId,
        -1
    );

    EXPECT_DOUBLE_EQ(
        cutVertex.sourceEdgeT,
        0.0
    );

    EXPECT_EQ(
        cutVertex.curveId,
        -1
    );

    EXPECT_EQ(
        cutVertex.cutPathOrder,
        -1
    );
}
