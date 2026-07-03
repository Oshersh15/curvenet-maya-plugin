#include "MayaMeshConverter.h"

#include <maya/MIntArray.h>
#include <maya/MPoint.h>

HalfEdgeMesh MayaMeshConverter::buildFromMayaMesh(const MFnMesh& meshFn)
{
    HalfEdgeMesh mesh;

    const int vertexCount = meshFn.numVertices();
    const int faceCount = meshFn.numPolygons();

    mesh.vertices.resize(vertexCount);

    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
    {
        MPoint position;
        meshFn.getPoint(vertexIndex, position, MSpace::kObject);

        mesh.vertices[vertexIndex].position = position;
        mesh.vertices[vertexIndex].outgoingHalfEdge = -1;
    }

    mesh.faces.resize(faceCount);

    for (int faceIndex = 0; faceIndex < faceCount; ++faceIndex)
    {
        MIntArray faceVertexIds;
        meshFn.getPolygonVertices(faceIndex, faceVertexIds);

        const int firstHalfEdgeIndex = static_cast<int>(mesh.halfEdges.size());
        mesh.faces[faceIndex].halfEdge = firstHalfEdgeIndex;

        const int vertexCountInFace = static_cast<int>(faceVertexIds.length());

        for (int localIndex = 0; localIndex < vertexCountInFace; ++localIndex)
        {
            const int startVertex = faceVertexIds[localIndex];
            const int endVertex = faceVertexIds[(localIndex + 1) % vertexCountInFace];

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
    }

    mesh.assignTwins();

    return mesh;
}
