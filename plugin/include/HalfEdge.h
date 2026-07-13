#pragma once

#include <vector>

struct Point3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Vertex
{
    Point3 position;
    int outgoingHalfEdge = -1;
};

struct HalfEdge
{
    int startVertex = -1;
    int endVertex = -1;
    int next = -1;
    int twin = -1;
    int face = -1;
};

struct Face
{
    int halfEdge = -1;
};

class HalfEdgeMesh
{
public:
    std::vector<Vertex> vertices;
    std::vector<HalfEdge> halfEdges;
    std::vector<Face> faces;

    void clear();

    int getNextHalfEdge(int halfEdgeIndex) const;
    int getTwinHalfEdge(int halfEdgeIndex) const;
    int getFaceOfHalfEdge(int halfEdgeIndex) const;
    int getStartVertex(int halfEdgeIndex) const;
    int getEndVertex(int halfEdgeIndex) const;

    std::vector<int> getAdjacentVertices(int vertexIndex) const;
    std::vector<int> getAdjacentFaces(int faceIndex) const;

    void createTestQuad();
    void createTwoQuadMesh();

    std::vector<int> traverseFace(int faceIndex) const;
    std::vector<int> getFaceHalfEdges(int faceIndex) const;

    double computeMeanEdgeLength() const;

    void assignTwins();

    std::vector<int> collectUniqueVerticesFromFaces(
        const std::vector<int>& faceIds
    ) const;
};
