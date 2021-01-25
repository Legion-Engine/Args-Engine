#pragma once
#include <core/core.hpp>
#include <physics/mesh_splitter_utils/mesh_splitter_typedefs.hpp>

namespace legion::physics
{
    struct BoundaryEdgeInfo
    {
        math::vec3 worldNormal;
        std::pair<math::vec3, math::vec3> intersectionPoints;
        std::pair<math::vec3, math::vec3> finalSortingDirection;
        std::vector<meshHalfEdgePtr> boundaryEdges;
        math::color drawColor;

        math::vec3 nextSupport;
        math::vec3 prevSupport;
        math::vec3 intersectionEdge;
        math::vec3 base;

    };

    //struct 

    struct MeshSplitterDebugHelper
    {
        std::pair<math::vec3, math::vec3> cuttingSetting;
        math::mat4 DEBUG_transform;
        std::vector<math::vec3> intersectionsPolygons;
        std::vector<math::vec3> nonIntersectionPolygons;
        std::vector< std::vector<math::vec3>> intersectionIslands;

        std::vector<BoundaryEdgeInfo> boundaryEdgesForPolygon;

        std::array< math::color, 5> colors =
        { math::colors::red,
            math::colors::cyan,
            math::colors::green,
            math::colors::magenta,
            math::colors::yellow
        };

        //std::vector

        int polygonCount;

        int polygonToDisplay = 0;

    };

    struct SplitterIntersectionInfo
    {
        SplitterIntersectionInfo(const math::vec3& pStartIntersection, const math::vec3& pStartToEndPosition,
            const math::vec2& pStartIntersectionUV, const math::vec2& pStartToEndUV) :

            startIntersectionPosition(pStartIntersection),
            startToEndPosition(pStartToEndPosition),
            startIntersectionUV(pStartIntersectionUV),
            startToEndUV(pStartToEndUV)
        {

        }

        auto GetIntersectionData() const
        {
            return std::make_tuple(startIntersectionPosition, startToEndPosition, startIntersectionUV, startToEndUV);
        }

        math::vec3 startIntersectionPosition;
        math::vec3 startToEndPosition;

        math::vec2 startIntersectionUV;
        math::vec2 startToEndUV;


    };

}

