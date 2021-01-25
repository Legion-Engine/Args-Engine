#pragma once
#include <core/core.hpp>


namespace legion::physics
{
	enum class SplitState
	{
		Above,
		Below,
		Split,
		Unknown
	};

	struct MeshHalfEdge;

	class SplittablePolygon : public std::enable_shared_from_this<SplittablePolygon>
	{
	public:
		SplittablePolygon(std::vector<std::shared_ptr<MeshHalfEdge>>& pEdgesInMesh,math::vec3 normal);

		std::vector<std::shared_ptr<MeshHalfEdge>>& GetMeshEdges();

        int CountBoundary();

		void CalculateLocalCentroid();

		void AssignEdgeOwnership();

		void ResetEdgeVisited();
		
		void CalculatePolygonSplit(const math::mat4& transform
			, math::vec3 planePosition, math::vec3 planeNormal,bool keepBelow);

		void IdentifyBoundaries(const math::mat4& transform);

		SplitState GetPolygonSplitState() const;

		//mostly used for debugging
		math::vec3 localCentroid{ 0,0,0 };
		math::color debugColor;

		bool isVisited = false;
		//bool isIntersectingPart = false;

		math::vec3 localNormal;
	private:

		SplitState m_SplitState = SplitState::Unknown;

		std::vector<std::shared_ptr<MeshHalfEdge>> edgesInPolygon;



	};
}


