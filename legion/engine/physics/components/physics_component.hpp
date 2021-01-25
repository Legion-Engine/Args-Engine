#pragma once

#include <physics/cube_collider_params.hpp>
#include <vector>
#include <physics/colliders/physicscollider.hpp>

namespace legion::physics
{
	struct physicsComponent
	{
        static void init(physicsComponent& comp)
        {
            comp.colliders = new std::vector<std::shared_ptr<PhysicsCollider>>();
        }

        static void destroy(physicsComponent& comp)
        {
            delete comp.colliders;
        }

		//physics material

        std::vector<std::shared_ptr<PhysicsCollider>>* colliders;

        bool isTrigger =false;

        math::vec3 localCenterOfMass{};

		//physics bitmask

        /** @brief given the colliders this physicsComponent, calculates the new local center of mass.
        * @note This is called internally by the physicsComponent every time a collider is added.
        */
        void calculateNewLocalCenterOfMass();

        /** @brief Instantiates a ConvexCollider and calls ConstructConvexHullWithMesh on it and passes the given mesh. This
         * ConvexCollider is then added to the list of PhysicsColliders
         * @param meshHandle - The mesh handle to lock the mesh and the mesh to create a hull from
        */
		std::shared_ptr<ConvexCollider> ConstructConvexHull(legion::core::mesh_handle& meshHandle,bool shouldDebug = false);

        void ConstructConvexHull(legion::core::mesh_handle& meshHandle, ConvexCollider& col);

        /** @brief Instantiates a ConvexCollider and calls ConstructBoxWithMesh on it and passes the given mesh. This
         * ConvexCollider is then added to the list of PhysicsColliders
        */
		void ConstructBox(/*mesh*/);

        /** @brief Instantiates a ConvexCollider and calls CreateBox on it and passes the given mesh. This 
         * ConvexCollider is then added to the list of PhysicsColliders
        */
		void AddBox(const cube_collider_params& cubeParams);

        /** @brief Instantiates a SphereCollider and creates a sphere that encompasses the given mesh. This
         * ConvexCollider is then added to the list of PhysicsColliders
        */
		void AddSphere(/*mesh*/);

	};
}


