#pragma once
#include <core/core.hpp>
#include <physics/mesh_splitter_utils/mesh_half_edge.hpp>
#include <physics/mesh_splitter_utils/primitive_mesh.hpp>
#include <physics/mesh_splitter_utils/splittable_polygon.hpp>

namespace legion::physics
{
    typedef std::pair<int, int> edgeVertexIndexPair;

    typedef std::map<edgeVertexIndexPair,
    std::shared_ptr< physics::MeshHalfEdge>> VertexIndexToHalfEdgePtr;

    typedef std::shared_ptr<MeshHalfEdge> meshHalfEdgePtr;
    typedef std::shared_ptr<SplittablePolygon> SplittablePolygonPtr;
}

