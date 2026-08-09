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

    for (int faceId = 0;
         faceId < static_cast<int>(
             curvenetCutResult.mesh.faces.size()
         );
         ++faceId)
    {
        const std::vector<int> halfEdgeIds =
            curvenetCutResult.mesh.traverseFace(faceId);

        if (halfEdgeIds.size() < 3)
        {
            return;
        }

        polygonCounts.append(
            static_cast<int>(halfEdgeIds.size())
        );

        for (int halfEdgeId : halfEdgeIds)
        {
            polygonVertexIds.append(
                curvenetCutResult.mesh
                    .halfEdges[halfEdgeId]
                    .startVertex
            );
        }
    }

    MStatus status;
    MFnTransform transformFn;
    MObject transformObject = transformFn.create(
        MObject::kNullObj,
        &status
    );

    if (!status)
    {
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
                meshFaceId < static_cast<int>(faceColors.length()))
            {
                faceColors[meshFaceId] = color;
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
