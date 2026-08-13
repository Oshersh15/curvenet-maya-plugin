#include "CurvenetRegionPreviewBuilder.h"

#include <maya/MColor.h>
#include <maya/MColorArray.h>
#include <maya/MFnMesh.h>
#include <maya/MFnTransform.h>
#include <maya/MGlobal.h>
#include <maya/MIntArray.h>
#include <maya/MObject.h>
#include <maya/MPointArray.h>
#include <maya/MString.h>

#include <cmath>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
constexpr int kRegionColorCount = 16;

MColor regionColor(int colorId)
{
    static const std::array<MColor, kRegionColorCount> colors = {
        MColor(0.95f, 0.18f, 0.18f, 1.0f),
        MColor(0.05f, 0.78f, 0.95f, 1.0f),
        MColor(0.25f, 0.90f, 0.18f, 1.0f),
        MColor(0.98f, 0.78f, 0.08f, 1.0f),
        MColor(0.55f, 0.20f, 0.95f, 1.0f),
        MColor(0.98f, 0.18f, 0.70f, 1.0f),
        MColor(1.00f, 0.45f, 0.05f, 1.0f),
        MColor(0.12f, 0.35f, 0.95f, 1.0f),
        MColor(0.05f, 0.65f, 0.30f, 1.0f),
        MColor(0.95f, 0.42f, 0.58f, 1.0f),
        MColor(0.05f, 0.70f, 0.68f, 1.0f),
        MColor(0.35f, 0.12f, 0.72f, 1.0f),
        MColor(0.72f, 0.55f, 0.05f, 1.0f),
        MColor(0.72f, 0.18f, 0.82f, 1.0f),
        MColor(0.95f, 0.32f, 0.12f, 1.0f),
        MColor(0.35f, 0.88f, 0.62f, 1.0f)
    };

    return colors[static_cast<std::size_t>(colorId) % colors.size()];
}

std::vector<int> buildRegionColorIds(
    const CurvenetCutResult& curvenetCutResult
)
{
    const int regionCount = static_cast<int>(
        curvenetCutResult.curvenetFaces.size()
    );
    std::vector<int> regionByMeshFace(
        curvenetCutResult.mesh.faces.size(),
        -1
    );
    std::vector<std::unordered_set<int>> neighbours(regionCount);

    for (int regionId = 0; regionId < regionCount; ++regionId)
    {
        for (int meshFaceId :
             curvenetCutResult.curvenetFaces[regionId].meshFaceIds)
        {
            if (meshFaceId >= 0 &&
                meshFaceId < static_cast<int>(regionByMeshFace.size()))
            {
                regionByMeshFace[meshFaceId] = regionId;
            }
        }
    }

    for (const HalfEdge& halfEdge : curvenetCutResult.mesh.halfEdges)
    {
        if (halfEdge.face < 0 ||
            halfEdge.face >= static_cast<int>(regionByMeshFace.size()) ||
            halfEdge.twin < 0 ||
            halfEdge.twin >= static_cast<int>(
                curvenetCutResult.mesh.halfEdges.size()
            ))
        {
            continue;
        }

        const int oppositeFaceId =
            curvenetCutResult.mesh.halfEdges[halfEdge.twin].face;

        if (oppositeFaceId < 0 ||
            oppositeFaceId >= static_cast<int>(regionByMeshFace.size()))
        {
            continue;
        }

        const int firstRegion = regionByMeshFace[halfEdge.face];
        const int secondRegion = regionByMeshFace[oppositeFaceId];

        if (firstRegion >= 0 && secondRegion >= 0 &&
            firstRegion != secondRegion)
        {
            neighbours[firstRegion].insert(secondRegion);
            neighbours[secondRegion].insert(firstRegion);
        }
    }

    std::vector<int> colorIds(regionCount, -1);

    for (int regionId = 0; regionId < regionCount; ++regionId)
    {
        std::unordered_set<int> unavailableColors;

        for (int neighbourId : neighbours[regionId])
        {
            if (neighbourId >= 0 && neighbourId < regionId &&
                colorIds[neighbourId] >= 0)
            {
                unavailableColors.insert(colorIds[neighbourId]);
            }
        }

        const int preferredColorId =
            (regionId * 7) % kRegionColorCount;
        int colorId = preferredColorId;

        for (int offset = 0; offset < kRegionColorCount; ++offset)
        {
            const int candidateColorId =
                (preferredColorId + offset) % kRegionColorCount;

            if (unavailableColors.count(candidateColorId) == 0)
            {
                colorId = candidateColorId;
                break;
            }
        }

        colorIds[regionId] = colorId;
    }

    return colorIds;
}
}

void CurvenetRegionPreviewBuilder::build(
    const std::string& ownerName,
    const CurvenetCutResult& curvenetCutResult
)
{
    if (curvenetCutResult.mesh.vertices.empty() ||
        curvenetCutResult.mesh.faces.empty())
    {
        return;
    }

    MPointArray points;
    points.setLength(
        static_cast<unsigned int>(
            curvenetCutResult.mesh.vertices.size()
        )
    );

    for (unsigned int vertexId = 0;
         vertexId < points.length();
         ++vertexId)
    {
        const Point3& position =
            curvenetCutResult.mesh.vertices[vertexId].position;
        points[vertexId] = MPoint(
            position.x,
            position.y,
            position.z
        );
    }

    MIntArray polygonCounts;
    MIntArray polygonVertexIds;
    std::vector<int> previewFaceIdByMeshFaceId(
        curvenetCutResult.mesh.faces.size(),
        -1
    );

    for (int faceId = 0;
         faceId < static_cast<int>(
             curvenetCutResult.mesh.faces.size()
         );
         ++faceId)
    {
        const std::vector<int> halfEdgeIds =
            curvenetCutResult.mesh.traverseFace(faceId);
        std::vector<int> faceVertexIds;
        faceVertexIds.reserve(halfEdgeIds.size());
        std::unordered_set<int> uniqueVertexIds;

        for (int halfEdgeId : halfEdgeIds)
        {
            const int vertexId =
                curvenetCutResult.mesh
                    .halfEdges[halfEdgeId]
                    .startVertex;

            if (vertexId < 0 ||
                vertexId >= static_cast<int>(points.length()))
            {
                faceVertexIds.clear();
                break;
            }

            faceVertexIds.push_back(vertexId);
            uniqueVertexIds.insert(vertexId);
        }

        /*
         * Cutting can leave zero-area slivers with fewer than three distinct
         * vertices. They have no visible surface, and Maya rejects the whole
         * preview mesh if they are submitted as polygons.
         */
        if (faceVertexIds.size() < 3 ||
            uniqueVertexIds.size() < 3)
        {
            continue;
        }

        previewFaceIdByMeshFaceId[faceId] =
            static_cast<int>(polygonCounts.length());
        polygonCounts.append(
            static_cast<int>(faceVertexIds.size())
        );

        for (int vertexId : faceVertexIds)
        {
            polygonVertexIds.append(vertexId);
        }
    }

    if (polygonCounts.length() == 0)
    {
        MGlobal::displayError(
            MString(
                "Curvenet preview failed: no valid mesh faces were produced."
            )
        );
        return;
    }

    MStatus status;
    MFnTransform transformFn;
    MObject transformObject = transformFn.create(
        MObject::kNullObj,
        &status
    );

    if (!status)
    {
        MGlobal::displayError(
            MString("Curvenet preview failed to create its transform: ") +
            status.errorString()
        );
        return;
    }

    const MString transformName(
        (ownerName + "_curvenet_regionPreview").c_str()
    );
    transformFn.setName(transformName);

    MFnMesh meshFn;
    MObject meshObject = meshFn.create(
        static_cast<int>(points.length()),
        static_cast<int>(polygonCounts.length()),
        points,
        polygonCounts,
        polygonVertexIds,
        transformObject,
        &status
    );

    if (!status || meshObject.isNull())
    {
        MGlobal::displayError(
            MString("Curvenet preview failed to create its mesh: ") +
            status.errorString()
        );
        return;
    }

    meshFn.setName(
        MString(
            (ownerName + "_curvenet_regionPreviewShape").c_str()
        )
    );

    const MString colorSetName("curvenetRegions");
    meshFn.createColorSetWithName(colorSetName);
    meshFn.setCurrentColorSetName(colorSetName);

    MColorArray faceColors;
    MIntArray faceIds;
    faceColors.setLength(polygonCounts.length());
    faceIds.setLength(polygonCounts.length());

    for (unsigned int faceId = 0;
         faceId < polygonCounts.length();
         ++faceId)
    {
        faceColors[faceId] =
            MColor(0.18f, 0.18f, 0.18f, 1.0f);
        faceIds[faceId] = static_cast<int>(faceId);
    }

    const std::vector<int> regionColorIds =
        buildRegionColorIds(curvenetCutResult);

    for (int curvenetFaceId = 0;
         curvenetFaceId < static_cast<int>(
             curvenetCutResult.curvenetFaces.size()
         );
         ++curvenetFaceId)
    {
        const MColor color =
            regionColor(regionColorIds[curvenetFaceId]);

        for (int meshFaceId :
             curvenetCutResult
                 .curvenetFaces[curvenetFaceId]
                 .meshFaceIds)
        {
            if (meshFaceId >= 0 &&
                meshFaceId < static_cast<int>(
                    previewFaceIdByMeshFaceId.size()
                ))
            {
                const int previewFaceId =
                    previewFaceIdByMeshFaceId[meshFaceId];

                if (previewFaceId >= 0)
                {
                    faceColors[previewFaceId] = color;
                }
            }
        }
    }

    meshFn.setFaceColors(
        faceColors,
        faceIds
    );
    meshFn.setDisplayColors(true);

    MGlobal::executeCommand(
        MString("parent -relative \"") + transformName +
            "\" \"" + ownerName.c_str() +
            "_curvenet_group\"; " +
            "sets -e -forceElement initialShadingGroup \"" +
            transformName + "\"; select -clear;",
        false,
        false
    );
}
