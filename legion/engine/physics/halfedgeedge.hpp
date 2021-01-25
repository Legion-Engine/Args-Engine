#pragma once
#include <core/core.hpp>
#include <physics/data/edge_label.hpp>
#include <physics/halfedgeface.hpp>
#include <rendering/debugrendering.hpp>

namespace legion::physics
{
	struct HalfEdgeEdge
	{
		HalfEdgeEdge* pairingEdge = nullptr;
		HalfEdgeEdge* nextEdge = nullptr;
		HalfEdgeEdge* prevEdge = nullptr;

		HalfEdgeFace* face = nullptr;

        EdgeLabel label;
        
		math::vec3 edgePosition;
        std::string id;

        HalfEdgeEdge() = default;

		HalfEdgeEdge(math::vec3 newEdgePositionPtr) : edgePosition{ newEdgePositionPtr }
		{

		}

		/**@brief sets 'nextEdge' and 'prevEdge' with the given HalfEdgeEdges
		*/
		void setNextAndPrevEdge(HalfEdgeEdge* newPrevEdge,HalfEdgeEdge* newNextEdge)
		{
			nextEdge = newNextEdge;
			prevEdge = newPrevEdge;
		}

        /**@brief Sets the pairingEdge to the passed in edge, and sets the edge.pairingEdge to this
         * if this edge or the passed edge already have pairing edge, the pairing edge of the pairing edge will be set to nullptr
         */
        void setPairingEdge(HalfEdgeEdge* edge)
        {
            pairingEdge = edge;
            edge->pairingEdge = this;
        }

		math::vec3 getLocalNormal() const
		{
			return face->normal;
		}

		/**@brief Gets the direction of the edge by getting the
		* vector starting from the current edge's position to the next edge
		*/
		math::vec3 getLocalEdgeDirection()  const
		{
			return nextEdge->edgePosition - edgePosition;
		}

        bool isVertexVisible(const math::vec3& vert)
        {
            float distanceToPlane =
                math::pointToPlane(vert, edgePosition, face->normal);

            return distanceToPlane > math::sqrt(math::epsilon<float>());
        }

        bool isEdgeHorizonFromVertex(const math::vec3& vert)
        {
            return isVertexVisible(vert) && !pairingEdge->isVertexVisible(vert);
        }


        void DEBUG_drawEdge(const math::mat4& transform, const math::color& debugColor,float time = 20.0f, float width = 5.0f)
        {
            math::vec3 worldStart = transform * math::vec4(edgePosition, 1);
            math::vec3 worldEnd = transform * math::vec4(nextEdge->edgePosition, 1);

            debug::user_projectDrawLine(worldStart, worldEnd, debugColor, width, time, true);
        }

        void DEBUG_drawInsetEdge(const math::vec3 spacing, const math::color& debugColor, float time = 20.0f, float width = 5.0f)
        {
            math::vec3 worldCentroid = face->centroid + spacing;

            math::vec3 worldStart = edgePosition + spacing;
            math::vec3 startDifference = (worldCentroid - worldStart) * 0.1f;
      
            math::vec3 worldEnd = nextEdge->edgePosition + spacing;
            math::vec3 endDifference = (worldCentroid - worldEnd) * 0.1f;


            debug::user_projectDrawLine(worldStart + startDifference, worldEnd + endDifference, debugColor, width, time, true);
        }


	};
}

