#include "MayaMeshConverter.h"

#include <maya/MIntArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>

MStatus MayaMeshConverter::buildFromMayaMesh(
    const MFnMesh& meshFn,
    HalfEdgeMesh& mesh
)
{
    mesh.clear();
    MStatus status;
    MPointArray points;
    MIntArray polygonCounts;
    MIntArray polygonVertexIds;

    status = meshFn.getPoints(points, MSpace::kObject);

    if (!status)
    {
        return status;
    }

    status = meshFn.getVertices(polygonCounts, polygonVertexIds);

    if (!status)
    {
        return status;
    }

    const int vertexCount = static_cast<int>(points.length());
    const int faceCount = static_cast<int>(polygonCounts.length());

    mesh.vertices.resize(vertexCount);

    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
    {
        const MPoint& position = points[vertexIndex];

        mesh.vertices[vertexIndex].position =
            Point3{position.x, position.y, position.z};
        mesh.vertices[vertexIndex].outgoingHalfEdge = -1;
    }

    mesh.faces.resize(faceCount);
    unsigned int polygonVertexOffset = 0;

    for (int faceIndex = 0; faceIndex < faceCount; ++faceIndex)
    {
        const int firstHalfEdgeIndex = static_cast<int>(mesh.halfEdges.size());
        mesh.faces[faceIndex].halfEdge = firstHalfEdgeIndex;

        const int vertexCountInFace = polygonCounts[faceIndex];

        if (vertexCountInFace < 3 ||
            polygonVertexOffset + vertexCountInFace > polygonVertexIds.length())
        {
            mesh.clear();
            return MS::kFailure;
        }

        for (int localIndex = 0; localIndex < vertexCountInFace; ++localIndex)
        {
            const int startVertex = polygonVertexIds[
                polygonVertexOffset + localIndex
            ];
            const int endVertex = polygonVertexIds[
                polygonVertexOffset +
                ((localIndex + 1) % vertexCountInFace)
            ];

            if (startVertex < 0 || startVertex >= vertexCount ||
                endVertex < 0 || endVertex >= vertexCount)
            {
                mesh.clear();
                return MS::kFailure;
            }

            HalfEdge halfEdge;
            halfEdge.startVertex = startVertex;
            halfEdge.endVertex = endVertex;
            halfEdge.face = faceIndex;
            halfEdge.twin = -1;
            halfEdge.next = firstHalfEdgeIndex + ((localIndex + 1) % vertexCountInFace);

            const int currentHalfEdgeIndex = static_cast<int>(mesh.halfEdges.size());
            mesh.halfEdges.push_back(halfEdge);

            if (mesh.vertices[startVertex].outgoingHalfEdge == -1)
            {
                mesh.vertices[startVertex].outgoingHalfEdge = currentHalfEdgeIndex;
            }
        }

        polygonVertexOffset += vertexCountInFace;
    }

    if (polygonVertexOffset != polygonVertexIds.length())
    {
        mesh.clear();
        return MS::kFailure;
    }

    mesh.assignTwins();

    return MS::kSuccess;
}
