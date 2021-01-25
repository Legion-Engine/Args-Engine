#pragma once
#include <core/core.hpp>
#include <physics/components/physics_component.hpp>
#include <physics/mesh_splitter_utils/mesh_splitter.hpp>
namespace legion::physics
{
    struct physics_manifold;

    struct FracturerColliderToMeshPairing
    {
        FracturerColliderToMeshPairing(
            std::shared_ptr<ConvexCollider> pColliderPair,
            ecs::component_handle<MeshSplitter> pMeshSplitterPairing) :
            colliderPair(pColliderPair) , meshSplitterPairing(pMeshSplitterPairing)
        {
            assert(pMeshSplitterPairing.entity);
        }

        std::shared_ptr<ConvexCollider> colliderPair;
        ecs::component_handle<MeshSplitter> meshSplitterPairing;

        void GenerateSplittingParamsFromCollider(std::shared_ptr<ConvexCollider> instantiatedCollider
            , std::vector<physics::MeshSplitParams>& meshSplitParams);
        

    };

	struct Fracturer
	{


		void HandleFracture(physics_manifold& manifold,bool& manifoldValid, bool isfracturingA);

        bool IsFractureConditionMet();

        void InitializeVoronoi(ecs::component_handle<physicsComponent> physicsComponent);

        void InvestigateColliderToMeshPairing(ecs::entity_handle ent
            ,std::vector<FracturerColliderToMeshPairing> colliderToMeshPairings);

        void InstantiateColliderMeshPairingWithEntity(ecs::entity_handle ent,
            std::vector< FracturerColliderToMeshPairing>& colliderToMeshPairings);

        int fractureCount = 0;

        std::vector<std::vector<math::vec3>> verticesList;
        std::vector<std::shared_ptr<ConvexCollider>> debugVectorcolliders;
        std::vector<math::mat4> transforms;
        static ecs::EcsRegistry* registry;
	};

   

}

