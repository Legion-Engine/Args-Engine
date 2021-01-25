#pragma once
#include <core/core.hpp>

namespace legion::physics
{
	struct HalfEdgeEdge;

	struct HalfEdgeFace
	{
        /**@class face_angle_relation
         * @brief Enum Struct to indicate the angle relation between two faces
         * Coplaner:    The faces could be merged, the angle between them is 180 degrees
         * Convex:      The faces are convex, the inside angle between them is less than 180 degrees, they create an acute angle
         * Concave:     The faces are concave, the inside angle is more than 180 degrees, they create an obtuse angle
         */
        enum struct face_angle_relation : int
        {
            coplanar = 0,
            convex,
            concave
        };

		HalfEdgeEdge* startEdge;
		math::vec3 normal;
		math::vec3 centroid;
        math::color DEBUG_color;
        
		HalfEdgeFace(HalfEdgeEdge* newStartEdge, math::vec3 newNormal);

        /**@brief Deletes all the edges of this face
         * Warning: pairing edges are not deleted because their face may still exist
         */
        void deleteEdges();

        /**@brief set the face of all the edges to this face
         */
        void setFaceForAllEdges();

		/** @brief given a function that takes in a HalfEdgeEdge*, 
		* executes the function on each edge connected to 'startEdge'
		*/
		void forEachEdge(legion::core::delegate< void(HalfEdgeEdge*)> functionToExecute);

        /**@brief Inverses the face
         * The edges will be stored in reverse and therefore the normal will point in the other direction
         * The normal will be changed 
         */
        void inverse();

        /**@brief Tests if this face is convex in regard to the passed face
         * @param other - the other plane
         * @return Returns true when convex, false if coplanar or concave
         */
        bool testConvexity(const HalfEdgeFace& other) const;

        /**@brief Makes the normals for this faces point to the convex side of the plane in regard to each other
         * @return Returns true when the face has been made convex, false when it was already convex
         */
        bool makeNormalsConvexWithFace(HalfEdgeFace& other);

        bool makeNormalsConvexWithNeighbors(HalfEdgeFace& other);

        /**@brief Calculates the angle relation from this to another face
         */
        face_angle_relation getAngleRelation(const HalfEdgeFace& other);

        /**@brief Tests if the the two planes are convex in regard to each other
         * @return true when convex, false if coplanar or concave
         */
        static bool testConvexity(const HalfEdgeFace& first, const HalfEdgeFace& second);

        /**@brief Makes the normals for both faces point to the convex side of the plane in regard to each other
         * @return Returns true when one of the faces or both have been made convex, false when they were already convex
         */
        static bool makeNormalsConvexWithFace(HalfEdgeFace& first, HalfEdgeFace& second);

        void mergeCoplanarNeighbors(std::vector<HalfEdgeFace*>& removed);

        /**@brief Finds the edge that splits the two faces
         * @return A pointer to the edge, nullptr if the two faces are not connected
         */
        static HalfEdgeEdge* findMiddleEdge(const HalfEdgeFace& first, const HalfEdgeFace& second);

        /**@brief Merges two faces
         * Warning: Only the face middleEdge.face will be usable after the merge, the other face will be deleted
         * Warning: The passed middleEdge will also be deleted
         * @param middleEdge The edge that seperates the two faces7
         * @return Pointer to the merged HalfEdgeFace
         */
        static HalfEdgeFace* mergeFaces(HalfEdgeEdge& middleEdge);

        friend bool operator==(const HalfEdgeFace& lhs, const HalfEdgeFace& rhs)
        {
            // Because the centroid is dependent on all edges, we do not need to check the edge positions
            return lhs.normal == rhs.normal && lhs.centroid == rhs.centroid;
        }

        static std::string to_string(const face_angle_relation& relation)
        {
            if (relation == face_angle_relation::coplanar) return "coplanar";
            if (relation == face_angle_relation::convex) return "convex";
            if (relation == face_angle_relation::concave) return "concave";
        }

        void DEBUG_DrawFace(const math::mat4& transform, const math::color& debugColor,  float time = 20.0f);

		~HalfEdgeFace();
	};
}
