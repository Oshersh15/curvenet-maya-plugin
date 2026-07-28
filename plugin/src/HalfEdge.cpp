#include "HalfEdge.h"

#include <cmath>

void HalfEdgeMesh::clear()
{
    vertices.clear();
    halfEdges.clear();
    faces.clear();
}

int HalfEdgeMesh::getNextHalfEdge(int halfEdgeIndex) const
{
    return halfEdges[halfEdgeIndex].next;
}

int HalfEdgeMesh::getTwinHalfEdge(int halfEdgeIndex) const
{
    return halfEdges[halfEdgeIndex].twin;
}

int HalfEdgeMesh::getFaceOfHalfEdge(int halfEdgeIndex) const
{
    return halfEdges[halfEdgeIndex].face;
}

int HalfEdgeMesh::getStartVertex(int halfEdgeIndex) const
{
    return halfEdges[halfEdgeIndex].startVertex;
}

int HalfEdgeMesh::getEndVertex(int halfEdgeIndex) const
{
    return halfEdges[halfEdgeIndex].endVertex;
}

std::vector<int> HalfEdgeMesh::getAdjacentVertices(int vertexIndex) const
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
        adjacentVertices.push_back(halfEdges[currentHalfEdge].endVertex);

        int twinHalfEdge = halfEdges[currentHalfEdge].twin;

        if (twinHalfEdge < 0 || twinHalfEdge >= static_cast<int>(halfEdges.size()))
        {
            break;
        }

        currentHalfEdge = halfEdges[twinHalfEdge].next;

    } while (currentHalfEdge != startHalfEdge && currentHalfEdge >= 0);

    return adjacentVertices;
}

std::vector<int> HalfEdgeMesh::getAdjacentFaces(int faceIndex) const
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

void HalfEdgeMesh::createTestQuad()
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

    halfEdges[0] = HalfEdge{0, 1, 1, -1, 0};
    halfEdges[1] = HalfEdge{1, 2, 2, -1, 0};
    halfEdges[2] = HalfEdge{2, 3, 3, -1, 0};
    halfEdges[3] = HalfEdge{3, 0, 0, -1, 0};

    faces[0].halfEdge = 0;
}

void HalfEdgeMesh::createTwoQuadMesh()
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

    halfEdges[0] = HalfEdge{0, 1, 1, -1, 0};
    halfEdges[1] = HalfEdge{1, 6, 2, -1, 0};
    halfEdges[2] = HalfEdge{6, 5, 3, -1, 0};
    halfEdges[3] = HalfEdge{5, 0, 0, -1, 0};

    halfEdges[4] = HalfEdge{1, 2, 5, -1, 1};
    halfEdges[5] = HalfEdge{2, 7, 6, -1, 1};
    halfEdges[6] = HalfEdge{7, 6, 7, -1, 1};
    halfEdges[7] = HalfEdge{6, 1, 4, -1, 1};

    assignTwins();
}

void HalfEdgeMesh::createFourQuadGrid()
{
    clear();

    vertices.resize(9);
    faces.resize(4);
    halfEdges.resize(16);

    vertices[0].position = Point3{0.0, 2.0, 0.0};
    vertices[1].position = Point3{1.0, 2.0, 0.0};
    vertices[2].position = Point3{2.0, 2.0, 0.0};

    vertices[3].position = Point3{0.0, 1.0, 0.0};
    vertices[4].position = Point3{1.0, 1.0, 0.0};
    vertices[5].position = Point3{2.0, 1.0, 0.0};

    vertices[6].position = Point3{0.0, 0.0, 0.0};
    vertices[7].position = Point3{1.0, 0.0, 0.0};
    vertices[8].position = Point3{2.0, 0.0, 0.0};

    /*
        Face 0:
        0 -> 1 -> 4 -> 3
    */
    halfEdges[0] = HalfEdge{0, 1, 1, -1, 0};
    halfEdges[1] = HalfEdge{1, 4, 2, -1, 0};
    halfEdges[2] = HalfEdge{4, 3, 3, -1, 0};
    halfEdges[3] = HalfEdge{3, 0, 0, -1, 0};

    /*
        Face 1:
        1 -> 2 -> 5 -> 4
    */
    halfEdges[4] = HalfEdge{1, 2, 5, -1, 1};
    halfEdges[5] = HalfEdge{2, 5, 6, -1, 1};
    halfEdges[6] = HalfEdge{5, 4, 7, -1, 1};
    halfEdges[7] = HalfEdge{4, 1, 4, -1, 1};

    /*
        Face 2:
        3 -> 4 -> 7 -> 6
    */
    halfEdges[8] = HalfEdge{3, 4, 9, -1, 2};
    halfEdges[9] = HalfEdge{4, 7, 10, -1, 2};
    halfEdges[10] = HalfEdge{7, 6, 11, -1, 2};
    halfEdges[11] = HalfEdge{6, 3, 8, -1, 2};

    /*
        Face 3:
        4 -> 5 -> 8 -> 7
    */
    halfEdges[12] = HalfEdge{4, 5, 13, -1, 3};
    halfEdges[13] = HalfEdge{5, 8, 14, -1, 3};
    halfEdges[14] = HalfEdge{8, 7, 15, -1, 3};
    halfEdges[15] = HalfEdge{7, 4, 12, -1, 3};

    faces[0].halfEdge = 0;
    faces[1].halfEdge = 4;
    faces[2].halfEdge = 8;
    faces[3].halfEdge = 12;

    vertices[0].outgoingHalfEdge = 0;
    vertices[1].outgoingHalfEdge = 1;
    vertices[2].outgoingHalfEdge = 5;
    vertices[3].outgoingHalfEdge = 3;
    vertices[4].outgoingHalfEdge = 2;
    vertices[5].outgoingHalfEdge = 6;
    vertices[6].outgoingHalfEdge = 11;
    vertices[7].outgoingHalfEdge = 10;
    vertices[8].outgoingHalfEdge = 14;

    assignTwins();
}

std::vector<int> HalfEdgeMesh::traverseFace(int faceIndex) const
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

std::vector<int> HalfEdgeMesh::getFaceHalfEdges(int faceIndex) const
{
    return traverseFace(faceIndex);
}

int HalfEdgeMesh::findOutgoingHalfEdgeInFace(
    int faceId,
    int vertexId
) const
{
    const std::vector<int> faceHalfEdges =
        getFaceHalfEdges(faceId);

    for (int halfEdgeId : faceHalfEdges)
    {
        if (halfEdgeId < 0 ||
            halfEdgeId >=
                static_cast<int>(halfEdges.size()))
        {
            continue;
        }

        if (halfEdges[halfEdgeId].startVertex ==
            vertexId)
        {
            return halfEdgeId;
        }
    }

    return -1;
}

int HalfEdgeMesh::findPreviousHalfEdgeInFace(
    int faceId,
    int halfEdgeId
) const
{
    const std::vector<int> faceHalfEdges =
        getFaceHalfEdges(faceId);

    for (int candidateHalfEdgeId :
         faceHalfEdges)
    {
        if (candidateHalfEdgeId < 0 ||
            candidateHalfEdgeId >=
                static_cast<int>(
                    halfEdges.size()
                ))
        {
            continue;
        }

        if (halfEdges[
                candidateHalfEdgeId
            ].next == halfEdgeId)
        {
            return candidateHalfEdgeId;
        }
    }

    return -1;
}

int HalfEdgeMesh::findFaceContainingVertices(
    int firstVertexId,
    int secondVertexId
) const
{
    if (firstVertexId < 0 ||
        firstVertexId >= static_cast<int>(vertices.size()) ||
        secondVertexId < 0 ||
        secondVertexId >= static_cast<int>(vertices.size()))
    {
        return -1;
    }

    for (int faceId = 0;
         faceId < static_cast<int>(faces.size());
         ++faceId)
    {
        const std::vector<int> faceHalfEdges =
            getFaceHalfEdges(faceId);

        bool firstFound = false;
        bool secondFound = false;

        for (int halfEdgeId : faceHalfEdges)
        {
            if (halfEdgeId < 0 ||
                halfEdgeId >=
                    static_cast<int>(halfEdges.size()))
            {
                continue;
            }

            const int startVertex =
                halfEdges[halfEdgeId].startVertex;

            if (startVertex == firstVertexId)
            {
                firstFound = true;
            }

            if (startVertex == secondVertexId)
            {
                secondFound = true;
            }
        }

        if (firstFound && secondFound)
        {
            return faceId;
        }
    }

    return -1;
}

double HalfEdgeMesh::computeMeanEdgeLength() const
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

        const Point3& startPoint = vertices[halfEdge.startVertex].position;
        const Point3& endPoint = vertices[halfEdge.endVertex].position;

        const double dx = endPoint.x - startPoint.x;
        const double dy = endPoint.y - startPoint.y;
        const double dz = endPoint.z - startPoint.z;

        totalLength += std::sqrt(dx * dx + dy * dy + dz * dz);
        ++edgeCount;
    }

    if (edgeCount == 0)
    {
        return 0.0;
    }

    return totalLength / static_cast<double>(edgeCount);
}

void HalfEdgeMesh::assignTwins()
{
    for (int firstIndex = 0;
         firstIndex < static_cast<int>(halfEdges.size());
         ++firstIndex)
    {
        for (int secondIndex = 0;
             secondIndex < static_cast<int>(halfEdges.size());
             ++secondIndex)
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

std::vector<int> HalfEdgeMesh::collectUniqueVerticesFromFaces(
    const std::vector<int>& faceIds
) const
{
    std::vector<int> uniqueVertexIds;

    for (int faceId : faceIds)
    {
        if (faceId < 0 || faceId >= static_cast<int>(faces.size()))
        {
            continue;
        }

        const std::vector<int> faceHalfEdges = traverseFace(faceId);

        for (int halfEdgeId : faceHalfEdges)
        {
            if (halfEdgeId < 0 || halfEdgeId >= static_cast<int>(halfEdges.size()))
            {
                continue;
            }

            const int vertexId = halfEdges[halfEdgeId].startVertex;

            if (vertexId < 0 || vertexId >= static_cast<int>(vertices.size()))
            {
                continue;
            }

            bool alreadyCollected = false;

            for (int existingVertexId : uniqueVertexIds)
            {
                if (existingVertexId == vertexId)
                {
                    alreadyCollected = true;
                    break;
                }
            }

            if (!alreadyCollected)
            {
                uniqueVertexIds.push_back(vertexId);
            }
        }
    }

    return uniqueVertexIds;
}

BoundaryHalfEdgeSplitResult
HalfEdgeMesh::splitBoundaryHalfEdge(
    int halfEdgeIndex,
    const Point3& cutPosition
)
{
    BoundaryHalfEdgeSplitResult result;

    if (halfEdgeIndex < 0 ||
        halfEdgeIndex >=
            static_cast<int>(halfEdges.size()))
    {
        return result;
    }

    HalfEdge& originalHalfEdge =
        halfEdges[halfEdgeIndex];

    if (originalHalfEdge.twin != -1)
    {
        return result;
    }

    if (originalHalfEdge.startVertex < 0 ||
        originalHalfEdge.endVertex < 0)
    {
        return result;
    }

    const int originalEndVertex =
        originalHalfEdge.endVertex;

    const int originalNext =
        originalHalfEdge.next;

    Vertex newVertex;
    newVertex.position =
        cutPosition;

    const int newVertexId =
        static_cast<int>(vertices.size());

    vertices.push_back(newVertex);

    HalfEdge secondHalfEdge;

    secondHalfEdge.startVertex =
        newVertexId;

    secondHalfEdge.endVertex =
        originalEndVertex;

    secondHalfEdge.next =
        originalNext;

    secondHalfEdge.face =
        originalHalfEdge.face;

    secondHalfEdge.twin = -1;

    const int secondHalfEdgeId =
        static_cast<int>(halfEdges.size());

    originalHalfEdge.endVertex =
        newVertexId;

    originalHalfEdge.next =
        secondHalfEdgeId;

    halfEdges.push_back(secondHalfEdge);

    vertices[newVertexId].outgoingHalfEdge =
        secondHalfEdgeId;

    result.success = true;

    result.newVertexId =
        newVertexId;

    result.firstHalfEdgeId =
        halfEdgeIndex;

    result.secondHalfEdgeId =
        secondHalfEdgeId;

    return result;
}

InternalHalfEdgeSplitResult
HalfEdgeMesh::splitInternalHalfEdge(
    int halfEdgeIndex,
    const Point3& cutPosition
)
{
    InternalHalfEdgeSplitResult result;

    if (halfEdgeIndex < 0 ||
        halfEdgeIndex >=
            static_cast<int>(halfEdges.size()))
    {
        return result;
    }

    const HalfEdge originalHalfEdge =
        halfEdges[halfEdgeIndex];

    const int twinHalfEdgeIndex =
        originalHalfEdge.twin;

    if (twinHalfEdgeIndex < 0 ||
        twinHalfEdgeIndex >=
            static_cast<int>(halfEdges.size()))
    {
        return result;
    }

    const HalfEdge originalTwinHalfEdge =
        halfEdges[twinHalfEdgeIndex];

    if (originalTwinHalfEdge.twin !=
        halfEdgeIndex)
    {
        return result;
    }

    if (originalHalfEdge.startVertex < 0 ||
        originalHalfEdge.endVertex < 0 ||
        originalTwinHalfEdge.startVertex < 0 ||
        originalTwinHalfEdge.endVertex < 0)
    {
        return result;
    }

    Vertex newVertex;
    newVertex.position =
        cutPosition;

    const int newVertexId =
        static_cast<int>(vertices.size());

    vertices.push_back(newVertex);

    const int firstNewHalfEdgeId =
        static_cast<int>(halfEdges.size());

    const int twinNewHalfEdgeId =
        firstNewHalfEdgeId + 1;

    HalfEdge firstNewHalfEdge;

    firstNewHalfEdge.startVertex =
        newVertexId;

    firstNewHalfEdge.endVertex =
        originalHalfEdge.endVertex;

    firstNewHalfEdge.next =
        originalHalfEdge.next;

    firstNewHalfEdge.face =
        originalHalfEdge.face;

    firstNewHalfEdge.twin =
        twinHalfEdgeIndex;

    HalfEdge twinNewHalfEdge;

    twinNewHalfEdge.startVertex =
        newVertexId;

    twinNewHalfEdge.endVertex =
        originalTwinHalfEdge.endVertex;

    twinNewHalfEdge.next =
        originalTwinHalfEdge.next;

    twinNewHalfEdge.face =
        originalTwinHalfEdge.face;

    twinNewHalfEdge.twin =
        halfEdgeIndex;

    halfEdges.push_back(
        firstNewHalfEdge
    );

    halfEdges.push_back(
        twinNewHalfEdge
    );

    halfEdges[halfEdgeIndex].endVertex =
        newVertexId;

    halfEdges[halfEdgeIndex].next =
        firstNewHalfEdgeId;

    halfEdges[halfEdgeIndex].twin =
        twinNewHalfEdgeId;

    halfEdges[twinHalfEdgeIndex].endVertex =
        newVertexId;

    halfEdges[twinHalfEdgeIndex].next =
        twinNewHalfEdgeId;

    halfEdges[twinHalfEdgeIndex].twin =
        firstNewHalfEdgeId;

    vertices[newVertexId].outgoingHalfEdge =
        firstNewHalfEdgeId;

    result.success = true;

    result.newVertexId =
        newVertexId;

    result.firstHalfEdgeId =
        halfEdgeIndex;

    result.firstNewHalfEdgeId =
        firstNewHalfEdgeId;

    result.twinHalfEdgeId =
        twinHalfEdgeIndex;

    result.twinNewHalfEdgeId =
        twinNewHalfEdgeId;

    return result;
}
