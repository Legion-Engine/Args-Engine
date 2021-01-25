#include <physics/mesh_splitter_utils/primitive_mesh.hpp>
#include <rendering/components/renderable.hpp>

namespace legion::physics
{
    ecs::EcsRegistry* PrimitiveMesh::m_ecs = nullptr;
    int PrimitiveMesh::count = 0;

    PrimitiveMesh::PrimitiveMesh
    (ecs::entity_handle pOriginalEntity, std::vector<std::shared_ptr<SplittablePolygon>>& pPolygons, rendering::material_handle pOriginalMaterial)
        : polygons(std::move(pPolygons)), originalMaterial(pOriginalMaterial), originalEntity(pOriginalEntity)
    {

    }

    ecs::entity_handle PrimitiveMesh::InstantiateNewGameObject()
    {
        auto [originalPosH, originalRotH, originalScaleH] = originalEntity.get_component_handles<transform>();
        math::mat4 trans = math::compose(originalScaleH.read(), originalRotH.read(), originalPosH.read());
      
        auto ent = m_ecs->createEntity();
        math::vec3 offset;

        mesh newMesh;

        math::vec3 scale = originalScaleH.read();
        populateMesh(newMesh, trans, offset, scale);
      
        newMesh.calculate_tangents(&newMesh);

        sub_mesh newSubMesh;
        newSubMesh.indexCount = newMesh.indices.size();
        newSubMesh.indexOffset = 0;

        newMesh.submeshes.push_back(newSubMesh);

        //creaate modelH
        mesh_handle meshH = core::MeshCache::create_mesh("newMesh" + std::to_string(count), newMesh);
        auto modelH = rendering::ModelCache::create_model(meshH);
        count++;

        //create renderable
        mesh_filter meshFilter = mesh_filter( meshH );

        ent.add_components<rendering::mesh_renderable>(meshFilter,rendering::mesh_renderer( originalMaterial));

        //create transform
      
        auto [posH, rotH ,scaleH] = m_ecs->createComponents<transform>(ent);
        math::vec3 newEntityPos = originalPosH.read() + offset;
        posH.write(newEntityPos);
        rotH.write(originalRotH.read());
        //scaleH.write(originalScaleH.read());

        math::vec3 initialPos = originalPosH.read();
        //+ math::vec3(7, 0, 0)
        originalPosH.write(initialPos );
        //m_ecs->destroyEntity(originalEntity);

        return ent;
    }

    void PrimitiveMesh::SetECSRegistry(ecs::EcsRegistry* ecs)
    {
        m_ecs = ecs;
    }



    void PrimitiveMesh::populateMesh(mesh& mesh, const math::mat4& originalTransform , math::vec3& outOffset,math::vec3& scale)
    {
        std::vector<uint>& indices = mesh.indices;
        std::vector<math::vec3>& vertices = mesh.vertices;
        std::vector<math::vec2>& uvs = mesh.uvs;
        std::vector<math::vec3>& normals = mesh.normals;

        //for each polygon in splittable polygon

        for (auto polygon : polygons)
        {
            if (polygon->GetMeshEdges().empty())
            {
                log::error("Primitive Mesh has has an empty polygon");
                continue;
            }

            polygon->ResetEdgeVisited();

            std::queue<meshHalfEdgePtr> unvisitedEdgeQueue;
            unvisitedEdgeQueue.push(polygon->GetMeshEdges().at(0));

            while (!unvisitedEdgeQueue.empty())
            {
                auto halfEdge = unvisitedEdgeQueue.front();
                unvisitedEdgeQueue.pop();

                if (!halfEdge->isVisited)
                {
                    halfEdge->markTriangleEdgeVisited();

                    auto [edge1, edge2, edge3] = halfEdge->getTriangle();

                    vertices.push_back(edge1->position);
                    vertices.push_back(edge2->position);
                    vertices.push_back(edge3->position);

                    uvs.push_back(edge1->uv);
                    uvs.push_back(edge2->uv);
                    uvs.push_back(edge3->uv);

                    if (!edge1->isBoundary && edge1->pairingEdge)
                    {
                        unvisitedEdgeQueue.push(edge1->pairingEdge);
                    }

                    if (!edge2->isBoundary && edge2->pairingEdge)
                    {
                        unvisitedEdgeQueue.push(edge2->pairingEdge);
                    }

                    if (!edge3->isBoundary && edge3->pairingEdge)
                    {
                        unvisitedEdgeQueue.push(edge3->pairingEdge);
                    }
                }
            }
        }

        //get centroid of vertices
        math::vec3 worldCentroid = math::vec3();

        for (const auto& vertex : vertices)
        {
            worldCentroid += vertex;
        }

        worldCentroid /= static_cast<float>(vertices.size());

        worldCentroid = originalTransform * math::vec4(worldCentroid, 1);
        math::vec3 originalPosition = originalTransform[3];

        outOffset = worldCentroid - originalPosition;

        math::vec3 localOffset = math::inverse(originalTransform) * math::vec4(outOffset, 0);

        //shift vertices by offset
        for (auto& vertex : vertices)
        {
            vertex -= localOffset;
            //vertex *= scale;
        }


        for (int i = 0; i < vertices.size(); i++)
        {
            indices.push_back(i);
        }

        //TODO stop generating fake normals
        for (int i = 0; i < vertices.size(); i++)
        {
            normals.push_back(math::vec3(0, 1,0));
        }
    }
}
