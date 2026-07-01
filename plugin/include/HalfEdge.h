#pragma once

#include <maya/MPoint.h>
#include <vector>

/*
    Stores one vertex in the half-edge mesh.

    A vertex stores:
    - its 3D position
    - one outgoing half-edge that starts from this vertex

    The outgoingHalfEdge value is an index into the halfEdges list.
    -1 means that no outgoing half-edge has been assigned yet.
*/
struct Vertex
{
    MPoint position;
    int outgoingHalfEdge = -1;
};

/*
    Stores one directed half-edge.

    A normal mesh edge is treated as two half-edges:
    one in each direction.

    For example:
        A -> B
        B -> A

    Each half-edge stores indices that allow the mesh to be traversed:
    - vertex: the vertex this half-edge points to
    - next: the next half-edge around the same face
    - twin: the opposite half-edge in the opposite direction
    - face: the face this half-edge belongs to

    -1 means that the relationship has not been assigned yet.
*/
struct HalfEdge
{
    int vertex = -1;
    int next = -1;
    int twin = -1;
    int face = -1;
};

/*
    Stores one polygon face in the half-edge mesh.

    A face only needs to store one half-edge belonging to it.
    From that half-edge, the full face boundary can be found by
    repeatedly following the next links.
*/
struct Face
{
    int halfEdge = -1;
};


/*
    Stores a complete half-edge mesh.

    This class owns the lists of:
    - vertices
    - half-edges
    - faces

    For now it only stores the data.
    Later it will contain functions for building the structure from a Maya mesh
    and traversing the mesh connectivity.
*/
class HalfEdgeMesh
{
public:
    std::vector<Vertex> vertices;
    std::vector<HalfEdge> halfEdges;
    std::vector<Face> faces;

    void clear()
    {
        vertices.clear();
        halfEdges.clear();
        faces.clear();
    }

    int getNextHalfEdge(int halfEdgeIndex) const
    {
        return halfEdges[halfEdgeIndex].next;
    }

    int getTwinHalfEdge(int halfEdgeIndex) const
    {
        return halfEdges[halfEdgeIndex].twin;
    }

    int getFaceOfHalfEdge(int halfEdgeIndex) const
    {
        return halfEdges[halfEdgeIndex].face;
    }

    int getVertexOfHalfEdge(int halfEdgeIndex) const
    {
        return halfEdges[halfEdgeIndex].vertex;
    }

    std::vector<int> getAdjacentVertices(int vertexIndex) const
    {
        std::vector<int> adjacentVertices;

        if (vertexIndex < 0 || vertexIndex >= static_cast<int>(vertices.size()))
        {
            return adjacentVertices;
        }

        int startHalfEdge = vertices[vertexIndex].outgoingHalfEdge;

        if (startHalfEdge < 0 || startHalfEdge >= static_cast<int>(halfEdges.size()))
        {
            return adjacentVertices;
        }

        int currentHalfEdge = startHalfEdge;

        do
        {
            int adjacentVertex = halfEdges[currentHalfEdge].vertex;
            adjacentVertices.push_back(adjacentVertex);

            int twinHalfEdge = halfEdges[currentHalfEdge].twin;

            if (twinHalfEdge < 0 || twinHalfEdge >= static_cast<int>(halfEdges.size()))
            {
                break;
            }

            currentHalfEdge = halfEdges[twinHalfEdge].next;

        } while (currentHalfEdge != startHalfEdge && currentHalfEdge >= 0);

        return adjacentVertices;
    }

    std::vector<int> getAdjacentFaces(int faceIndex) const
    {
        std::vector<int> adjacentFaces;

        if (faceIndex < 0 || faceIndex >= static_cast<int>(faces.size()))
        {
            return adjacentFaces;
        }

        int startHalfEdge = faces[faceIndex].halfEdge;

        if (startHalfEdge < 0 || startHalfEdge >= static_cast<int>(halfEdges.size()))
        {
            return adjacentFaces;
        }

        int currentHalfEdge = startHalfEdge;

        do
        {
            int twinHalfEdge = halfEdges[currentHalfEdge].twin;

            if (twinHalfEdge >= 0 && twinHalfEdge < static_cast<int>(halfEdges.size()))
            {
                int adjacentFace = halfEdges[twinHalfEdge].face;

                if (adjacentFace >= 0 && adjacentFace != faceIndex)
                {
                    adjacentFaces.push_back(adjacentFace);
                }
            }

            currentHalfEdge = halfEdges[currentHalfEdge].next;

        } while (currentHalfEdge != startHalfEdge && currentHalfEdge >= 0);

        return adjacentFaces;
    }

    /* temporary test. later -> buildFromMayaMesh(MObject mesh) */
    void createTestQuad()
    {
        clear();

        vertices.resize(4);
        halfEdges.resize(4);
        faces.resize(1);

        vertices[0].position = MPoint(0.0, 1.0, 0.0);
        vertices[1].position = MPoint(1.0, 1.0, 0.0);
        vertices[2].position = MPoint(1.0, 0.0, 0.0);
        vertices[3].position = MPoint(0.0, 0.0, 0.0);

        vertices[0].outgoingHalfEdge = 0;
        vertices[1].outgoingHalfEdge = 1;
        vertices[2].outgoingHalfEdge = 2;
        vertices[3].outgoingHalfEdge = 3;

        halfEdges[0].vertex = 1;
        halfEdges[0].next = 1;
        halfEdges[0].twin = -1;
        halfEdges[0].face = 0;

        halfEdges[1].vertex = 2;
        halfEdges[1].next = 2;
        halfEdges[1].twin = -1;
        halfEdges[1].face = 0;

        halfEdges[2].vertex = 3;
        halfEdges[2].next = 3;
        halfEdges[2].twin = -1;
        halfEdges[2].face = 0;

        halfEdges[3].vertex = 0;
        halfEdges[3].next = 0;
        halfEdges[3].twin = -1;
        halfEdges[3].face = 0;

        faces[0].halfEdge = 0;
    }

    std::vector<int> traverseFace(int faceIndex) const
    {
        std::vector<int> traversal;

        if (faceIndex < 0 || faceIndex >= static_cast<int>(faces.size()))
        {
            return traversal;
        }

        int startHalfEdge = faces[faceIndex].halfEdge;

        if (startHalfEdge < 0 || startHalfEdge >= static_cast<int>(halfEdges.size()))
        {
            return traversal;
        }

        int currentHalfEdge = startHalfEdge;

        do
        {
            traversal.push_back(currentHalfEdge);
            currentHalfEdge = halfEdges[currentHalfEdge].next;

        } while (currentHalfEdge != startHalfEdge &&
                 currentHalfEdge >= 0 &&
                 currentHalfEdge < static_cast<int>(halfEdges.size()));

        return traversal;
    }
};
