#pragma once

#include <core/core.hpp>
#include <physics/data/physics_manifold.hpp>

namespace legion::physics
{
	//After a collision is found, stores the necessary information to create contact points
	class PenetrationQuery
	{
	public:

		math::vec3 faceCentroid,normal;
		float penetration = 0.0f;
		bool isARef;
        std::string debugID = "na";

		PenetrationQuery(math::vec3& pFaceCentroid,math::vec3& pNormal,float pPenetration,bool pIsARef) :
			faceCentroid(pFaceCentroid),normal(pNormal),penetration(pPenetration),isARef(pIsARef)
		{

		}

		/** @brief given the necessary information to create contact points between 2 colliding colliders
		 * calculates the contact points of the colliders and pushes them into the physics_manifold
		 */
		virtual void populateContactList(physics_manifold& manifold
			,math::mat4& refTransform,math::mat4 incTransform,std::shared_ptr<PhysicsCollider> refCollider)
		{

		}
	};

}

