#pragma once

#include <vector>
#include <cmath>

/*
    This structure stores vertex positions independently from
    the Maya API to allow unit testing of the HalfEdgeMesh.
*/
struct Point3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

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
    Point3 position;
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
    int startVertex = -1; /* where the directed edge starts. */
    int endVertex = -1;   /* where the directed edge points to. */
    int next = -1;        /* the next half-edge around the face. */
    int twin = -1;        /* the opposite half-edge. */
    int face = -1;        /* the face this half-edge belongs to. */
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

    int getStartVertex(int halfEdgeIndex) const
    {
        return halfEdges[halfEdgeIndex].startVertex;
    }

    int getEndVertex(int halfEdgeIndex) const
    {
        return halfEdges[halfEdgeIndex].endVertex;
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
            int adjacentVertex = halfEdges[currentHalfEdge].endVertex;
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

        vertices[0].position = Point3{0.0, 1.0, 0.0};
        vertices[1].position = Point3{1.0, 1.0, 0.0};
        vertices[2].position = Point3{1.0, 0.0, 0.0};
        vertices[3].position = Point3{0.0, 0.0, 0.0};

        vertices[0].outgoingHalfEdge = 0;
        vertices[1].outgoingHalfEdge = 1;
        vertices[2].outgoingHalfEdge = 2;
        vertices[3].outgoingHalfEdge = 3;

        halfEdges[0].startVertex = 0;
        halfEdges[0].endVertex = 1;
        halfEdges[0].next = 1;
        halfEdges[0].twin = -1;
        halfEdges[0].face = 0;

        halfEdges[1].startVertex = 1;
        halfEdges[1].endVertex = 2;
        halfEdges[1].next = 2;
        halfEdges[1].twin = -1;
        halfEdges[1].face = 0;

        halfEdges[2].startVertex = 2;
        halfEdges[2].endVertex = 3;
        halfEdges[2].next = 3;
        halfEdges[2].twin = -1;
        halfEdges[2].face = 0;

        halfEdges[3].startVertex = 3;
        halfEdges[3].endVertex = 0;
        halfEdges[3].next = 0;
        halfEdges[3].twin = -1;
        halfEdges[3].face = 0;

        faces[0].halfEdge = 0;
    }

    void createTwoQuadMesh()
    {
        clear();

        vertices.resize(8);
        faces.resize(2);
        halfEdges.resize(8);

        vertices[0].position = Point3{0.0, 1.0, 0.0};
        vertices[1].position = Point3{1.0, 1.0, 0.0};
        vertices[2].position = Point3{2.0, 1.0, 0.0};

        vertices[5].position = Point3{0.0, 0.0, 0.0};
        vertices[6].position = Point3{1.0, 0.0, 0.0};
        vertices[7].position = Point3{2.0, 0.0, 0.0};

        faces[0].halfEdge = 0;
        faces[1].halfEdge = 4;

        // Face 0

        halfEdges[0].startVertex = 0;
        halfEdges[0].endVertex = 1;
        halfEdges[0].next = 1;
        halfEdges[0].face = 0;

        halfEdges[1].startVertex = 1;
        halfEdges[1].endVertex = 6;
        halfEdges[1].next = 2;
        halfEdges[1].face = 0;

        halfEdges[2].startVertex = 6;
        halfEdges[2].endVertex = 5;
        halfEdges[2].next = 3;
        halfEdges[2].face = 0;

        halfEdges[3].startVertex = 5;
        halfEdges[3].endVertex = 0;
        halfEdges[3].next = 0;
        halfEdges[3].face = 0;

        // Face 1

        halfEdges[4].startVertex = 1;
        halfEdges[4].endVertex = 2;
        halfEdges[4].next = 5;
        halfEdges[4].face = 1;

        halfEdges[5].startVertex = 2;
        halfEdges[5].endVertex = 7;
        halfEdges[5].next = 6;
        halfEdges[5].face = 1;

        halfEdges[6].startVertex = 7;
        halfEdges[6].endVertex = 6;
        halfEdges[6].next = 7;
        halfEdges[6].face = 1;

        halfEdges[7].startVertex = 6;
        halfEdges[7].endVertex = 1;
        halfEdges[7].next = 4;
        halfEdges[7].face = 1;

        assignTwins();
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

    std::vector<int> getFaceHalfEdges(int faceIndex) const
    {
        std::vector<int> faceHalfEdges;

        if (faceIndex < 0 || faceIndex >= static_cast<int>(faces.size()))
        {
            return faceHalfEdges;
        }

        int startHalfEdge = faces[faceIndex].halfEdge;

        if (startHalfEdge < 0 || startHalfEdge >= static_cast<int>(halfEdges.size()))
        {
            return faceHalfEdges;
        }

        int currentHalfEdge = startHalfEdge;

        do
        {
            faceHalfEdges.push_back(currentHalfEdge);
            currentHalfEdge = halfEdges[currentHalfEdge].next;

        } while (currentHalfEdge != startHalfEdge &&
                 currentHalfEdge >= 0 &&
                 currentHalfEdge < static_cast<int>(halfEdges.size()));

        return faceHalfEdges;
    }

    double computeMeanEdgeLength() const
    {
        double totalLength = 0.0;
        int edgeCount = 0;

        for (int halfEdgeIndex = 0;
             halfEdgeIndex < static_cast<int>(halfEdges.size());
             ++halfEdgeIndex)
        {
            const HalfEdge& halfEdge = halfEdges[halfEdgeIndex];

            if (halfEdge.startVertex < 0 ||
                halfEdge.startVertex >= static_cast<int>(vertices.size()) ||
                halfEdge.endVertex < 0 ||
                halfEdge.endVertex >= static_cast<int>(vertices.size()))
            {
                continue;
            }

            if (halfEdge.twin >= 0 && halfEdgeIndex > halfEdge.twin)
            {
                continue;
            }

            const Point3& startPoint =
                vertices[halfEdge.startVertex].position;

            const Point3& endPoint =
                vertices[halfEdge.endVertex].position;

            const double dx = endPoint.x - startPoint.x;
            const double dy = endPoint.y - startPoint.y;
            const double dz = endPoint.z - startPoint.z;

            const double edgeLength =
                std::sqrt(dx * dx + dy * dy + dz * dz);

            totalLength += edgeLength;
            ++edgeCount;
        }

        if (edgeCount == 0)
        {
            return 0.0;
        }

        return totalLength / static_cast<double>(edgeCount);
    }

    void assignTwins()
    {
        for (int firstIndex = 0; firstIndex < static_cast<int>(halfEdges.size()); ++firstIndex)
        {
            for (int secondIndex = 0; secondIndex < static_cast<int>(halfEdges.size()); ++secondIndex)
            {
                if (firstIndex == secondIndex)
                {
                    continue;
                }

                HalfEdge& firstEdge = halfEdges[firstIndex];
                const HalfEdge& secondEdge = halfEdges[secondIndex];

                if (firstEdge.startVertex == secondEdge.endVertex &&
                    firstEdge.endVertex == secondEdge.startVertex)
                {
                    firstEdge.twin = secondIndex;
                    break;
                }
            }
        }
    }
};
