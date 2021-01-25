#pragma once
#include <core/core.hpp>
#include <physics/components/physics_component.hpp>

namespace legion::physics
{
	/** @struct physics_manifold_precursor
	* @brief contains the necessary data to create a physics_manifold
	*/
	struct physics_manifold_precursor
	{
		math::mat4 worldTransform;
		ecs::component_handle<physicsComponent> physicsComponentHandle;
		int id;



		physics_manifold_precursor(math::mat4 pWorldTransform,ecs::component_handle<physicsComponent>& pPhysicsCompHandle,int precursorID) :
			worldTransform(pWorldTransform), physicsComponentHandle(pPhysicsCompHandle),id(precursorID)
		{

		}
	};

}
