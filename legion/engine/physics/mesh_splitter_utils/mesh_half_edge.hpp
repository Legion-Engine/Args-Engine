#pragma once
#include <core/core.hpp>
#include <physics/mesh_splitter_utils/splittable_polygon.hpp>
#include <physics/physics_statics.hpp>

namespace legion::physics
{
    struct MeshHalfEdge : std::enable_shared_from_this<MeshHalfEdge>
    {
        math::vec3 position;
        math::vec2 uv;

        std::shared_ptr<MeshHalfEdge> nextEdge = nullptr;
        std::shared_ptr<MeshHalfEdge> pairingEdge = nullptr;
        std::shared_ptr<MeshHalfEdge> shadowEdge = nullptr;

        std::weak_ptr<SplittablePolygon> owner;

        //TODO: these bools should probably be stored as an array in SplittablePolygon
        bool isVisited = false;
        bool isBoundary = false;

        MeshHalfEdge(const math::vec3& pPosition, const math::vec2& pUVs, std::weak_ptr<SplittablePolygon> pOwner) : position(pPosition), uv(pUVs),owner(pOwner)
        {

        }

        MeshHalfEdge(const math::vec3& pPosition,const math::vec2& pUVs) : position(pPosition),uv(pUVs)
        {

        }

        MeshHalfEdge(const math::vec3& pPosition) : position(pPosition)
        {

        }

        auto getTriangle() 
        {
            return std::make_tuple(shared_from_this(), nextEdge, nextEdge->nextEdge);
        }

        auto getShadowTriangle()
        {
            return std::make_tuple(shadowEdge, nextEdge->shadowEdge, nextEdge->nextEdge->shadowEdge);
        }

        void setPairing(std::shared_ptr<MeshHalfEdge>& newPairing)
        {
            pairingEdge = newPairing;
            newPairing->pairingEdge = shared_from_this();
        }

        void cloneOnShadowEdge()
        {
            shadowEdge = std::make_shared<MeshHalfEdge>(position, uv);
            shadowEdge->isBoundary = isBoundary;
        }

        static void connectIntoTriangle(std::shared_ptr<MeshHalfEdge> first
            , std::shared_ptr<MeshHalfEdge> second,
            std::shared_ptr<MeshHalfEdge> third)
        {
            first->nextEdge = second;
            second->nextEdge = third;
            third->nextEdge = first;
        }
 

        bool isSplitByPlane(const math::mat4& transform
            , const math::vec3& planePosition
            ,const  math::vec3& planeNormal)
        {
            int x = 0;
            int y = 0;

            auto [currentDistFromPlane, nextDistFromPlane] = getEdgeDistancesFromPlane(transform, planePosition, planeNormal);

            static float splitEpsilon = math::sqrt(math::epsilon<float>());

            if (currentDistFromPlane > splitEpsilon)
            {
                x = 1;
            }
            else if (currentDistFromPlane < -splitEpsilon)
            {
                x = -1;
            }


            if (nextDistFromPlane > splitEpsilon)
            {
                y = 1;
            }
            else if (nextDistFromPlane < -splitEpsilon)
            {
                y = -1;
            }
            //log::debug("x {} y {}", x, y);
            int edgeSplitState = x * y;


            return edgeSplitState < 0;


        }

        /** @brief Given the transform of the entity associated with this edge, returns
       *  the centroid of the edge in world space.
       */
        math::vec3 getWorldCentroid(const math::mat4& transform) const
        {
            return transform * math::vec4((position + nextEdge->position) * 0.5f, 1);
        }

        /** @brief Given the transform of the entity associated with this edge and the position and normal of the cutting plane,
        * check if one of the vertices of the is above the plane
        */
        bool isEdgePartlyAbovePlane(const math::mat4& transform, const math::vec3& planePosition, const math::vec3& planeNormal)
        {
            auto [currentDistFromPlane, nextDistFromPlane] = getEdgeDistancesFromPlane(transform, planePosition, planeNormal);

            bool currentAbovePlane = currentDistFromPlane > constants::polygonSplitterEpsilon;
            bool nextAbovePlane = nextDistFromPlane > constants::polygonSplitterEpsilon;

            return currentAbovePlane || nextAbovePlane;
        }

        /** @brief Given the transform of the entity associated with this edge and the position and normal of the cutting plane,
        * check if one of the vertices of the is below the plane
        */
        bool isEdgePartlyBelowPlane(const math::mat4& transform, const math::vec3& planePosition, const math::vec3& planeNormal)
        {
            auto [currentDistFromPlane, nextDistFromPlane] = getEdgeDistancesFromPlane(transform, planePosition, planeNormal);

            bool currentBelowPlane = currentDistFromPlane < -constants::polygonSplitterEpsilon;
            bool nextBelowPlane = nextDistFromPlane < -constants::polygonSplitterEpsilon;

            return currentBelowPlane || nextBelowPlane;
        }

        /** @brief Given the transform of the entity associated with this edge and the position and normal of the cutting plane,
        *  Gets Edge Distances From Plane
        */
        std::tuple<float, float> getEdgeDistancesFromPlane(const math::mat4& transform, const math::vec3& planePosition, const math::vec3& planeNormal)
        {
            auto [currentWorldPos, nextWorldPos] = getEdgeWorldPositions(transform);

            float currentDistFromPlane = PhysicsStatics::PointDistanceToPlane(planeNormal, planePosition,  currentWorldPos);
            float nextDistFromPlane = PhysicsStatics::PointDistanceToPlane(planeNormal, planePosition,  nextWorldPos);

            return std::make_tuple(currentDistFromPlane, nextDistFromPlane);
        }

        /** @brief Given the transform of the entity associated with this edge, returns 
        *  the 2 vertices of the edge in world space
        */
        std::tuple<math::vec3, math::vec3> getEdgeWorldPositions(const math::mat4& transform)
        {
            math::vec3 currentWorldPos = getEdgeWorldPosition(transform);
            math::vec3 nextWorldPos = nextEdge->getEdgeWorldPosition(transform);

            return std::make_tuple(currentWorldPos, nextWorldPos);
        }

        /** @brief Given the transform of the entity associated with this edge, returns
        *  'position' in world space.
        */
        math::vec3 getEdgeWorldPosition(const math::mat4& transform)
        {
            return transform * math::vec4(position, 1);
        }

        bool attemptGetTrianglesInEdges
        (std::shared_ptr<MeshHalfEdge>& nextEdge, std::shared_ptr<MeshHalfEdge>& prevEdge)
        {
            if (!this->nextEdge)
            {
                return false;
            }

            nextEdge = this->nextEdge;

            if (!nextEdge->nextEdge)
            {
                return false;
            }

            prevEdge = nextEdge->nextEdge;

            return true;
        }

        void markTriangleEdgeVisited()
        {
            isVisited = true;
            nextEdge->isVisited = true;
            nextEdge->nextEdge->isVisited = true;
        }

        void populateQueueWithTriangleNeighbor(std::queue<std::shared_ptr<MeshHalfEdge>>& edgeQueue)
        {
            if (pairingEdge)
            {
                edgeQueue.push(pairingEdge);
            }

            if (nextEdge->pairingEdge)
            {
                edgeQueue.push(nextEdge->pairingEdge);
            }

            if (nextEdge->nextEdge->pairingEdge)
            {
                edgeQueue.push(nextEdge->nextEdge->pairingEdge);
            }

        }

        void populateVectorWithTriangle(std::vector<std::shared_ptr<MeshHalfEdge>>& edgeVector)
        {
            edgeVector.push_back(shared_from_this());
            edgeVector.push_back(nextEdge);
            edgeVector.push_back(nextEdge->nextEdge);
        }

        bool isTriangleValid()
        {
            std::shared_ptr<MeshHalfEdge> nextEdge = nullptr;
            std::shared_ptr<MeshHalfEdge> prevEdge = nullptr;
            return attemptGetTrianglesInEdges( nextEdge, prevEdge);
        }

        math::vec3 calculateEdgeNormal(const math::mat4& transform)
        {
            math::vec3 firstDir = transform * math::vec4(nextEdge->position - position,0);
            math::vec3 secondDir = transform * math::vec4(nextEdge->nextEdge->position - position,0);

            return math::normalize(math::cross(firstDir, secondDir));
        }

        bool isNormalCloseEnough(const math::vec3& comparisonNormal, const math::mat4& transform)
        {
            return compareNormals(calculateEdgeNormal(transform), comparisonNormal);
        }

        static bool compareNormals(const math::vec3& comparisonNormal, const math::vec3& otherNormal) 
        {
            static float toleranceDot = math::cos(math::deg2rad(5.0f));

            float dotResult = math::dot(otherNormal, comparisonNormal);

            return dotResult > toleranceDot;
        }

    };
}
