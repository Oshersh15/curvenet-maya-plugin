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
#include <unordered_set>
#include <vector>

namespace
{
MColor regionColor(int regionId)
{
    const float hue = std::fmod(
        static_cast<float>(regionId) * 0.61803398875f,
        1.0f
    );
    const float saturation = 0.72f;
    const float value = 0.95f;
    const float scaledHue = hue * 6.0f;
    const int sector = static_cast<int>(std::floor(scaledHue));
    const float fraction = scaledHue - static_cast<float>(sector);
    const float minimum = value * (1.0f - saturation);
    const float descending = value * (1.0f - saturation * fraction);
    const float ascending = value * (1.0f - saturation * (1.0f - fraction));

    switch (sector % 6)
    {
        case 0: return MColor(value, ascending, minimum, 1.0f);
        case 1: return MColor(descending, value, minimum, 1.0f);
        case 2: return MColor(minimum, value, ascending, 1.0f);
        case 3: return MColor(minimum, descending, value, 1.0f);
        case 4: return MColor(ascending, minimum, value, 1.0f);
        default: return MColor(value, minimum, descending, 1.0f);
    }
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

    for (int curvenetFaceId = 0;
         curvenetFaceId < static_cast<int>(
             curvenetCutResult.curvenetFaces.size()
         );
         ++curvenetFaceId)
    {
        const MColor color =
            regionColor(curvenetFaceId);

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
