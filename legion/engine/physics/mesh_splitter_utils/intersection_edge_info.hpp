#pragma once
#include <core/core.hpp>
#include <physics/mesh_splitter_utils/mesh_splitter_typedefs.hpp>
namespace legion::physics
{
    /** @struct IntersectionEdgeInfo
    * @brief The information required to create a edge for a hole polygon
    */
	struct IntersectionEdgeInfo
	{
		IntersectionEdgeInfo(meshHalfEdgePtr pPairintToConnectTo) : 
			pairingToConnectTo(pPairintToConnectTo)
		{
			first = pairingToConnectTo->nextEdge->position;
			second = pairingToConnectTo->position;
		}


		//one of the edge created by the intersection of a polygon
		meshHalfEdgePtr pairingToConnectTo = nullptr;

		//the instantiated edge that will be the pairing of pairingToConnectTo
		meshHalfEdgePtr instantiatedEdge = nullptr;

		//the instantiated edge that will be located at the centroid of the intersecting polygon
		meshHalfEdgePtr centroidEdge = nullptr;

		//the position of the nextEdge of 'pairingToConnectTo'
		math::vec3 first;
		//the position of the edge of 'pairingToConnectTo'
		math::vec3 second;

		
	};



	
}

