#pragma once
#include <physics/mesh_splitter_utils/mesh_splitter_typedefs.hpp>


namespace legion::physics
{
    /** @struct HalfEdgeFinder
    * @brief Responsible for creating a half-edge data structure for a mesh
    */
	struct HalfEdgeFinder
	{
        meshHalfEdgePtr currentPtr;

        /** @brief Given a mesh and its associated transform,
        * populates the meshHalfEdges queue with the created meshHalfEdgePtrs
        */
        void FindHalfEdge
        (mesh& mesh, const math::mat4& transform,
            std::queue<meshHalfEdgePtr>& meshHalfEdges)
        {
            VertexIndexToHalfEdgePtr indexToEdgeMap;

            //[1] find the unique vertices of a mesh. We are currently keeping a list of found
            //vertices and comparing them to our current vertices

            // holds the "pointer" to the unique indices inside mesh->Indices
            std::vector<int> uniqueIndex;

            // stores the unique vertices of the mesh
            std::vector<math::vec3> uniquePositions;

            int uniqueIndexCount = -1;

            auto& vertices = mesh.vertices;
            auto& indices = mesh.indices;

            for (int i = 0; i < vertices.size(); ++i)
            {
                math::vec3 position = vertices.at(i);

                bool isVectorSeen = false;

                //have we found this vector before?
                for (int j = 0; j < uniquePositions.size(); j++)
                {
                    math::vec3 transformedPos = transform * math::vec4(position, 1);
                    math::vec3 uniqueTransformedPos = transform * math::vec4(uniquePositions[j], 1);

                    float dist = math::distance(uniqueTransformedPos, transformedPos);

                    if (math::epsilonEqual<float>(dist, 0.0f, math::epsilon<float>()))
                    {
                        //we have seen this vector before
                        uniqueIndex.push_back(j);
                        isVectorSeen = true;
                        break;
                    }
                }

                if (!isVectorSeen)
                {
                    //we have not seen this position before,add it 
                    uniqueIndexCount++;
                    uniqueIndex.push_back(uniqueIndexCount);

                    uniquePositions.push_back(position);

                }
            }


            //[2] use the unique vertices of a mesh to generate half-edge data structure

            for (int i = 0; i < indices.size(); i += 3)
            {
                int firstVertIndex = indices.at(i);
                int secondVertIndex = indices.at(i + 1);
                int thirdVertIndex = indices.at(i + 2);

                int uniqueFirstIndex = uniqueIndex.at(firstVertIndex);
                int uniqueSecondIndex = uniqueIndex.at(secondVertIndex);
                int uniqueThirdIndex = uniqueIndex.at(thirdVertIndex);

                //-----------------instantiate first half edge---------------------//
                //MeshHalfEdge firstEdge = std::make
                auto firstEdge = InstantiateEdge(firstVertIndex
                    , edgeVertexIndexPair(uniqueFirstIndex, uniqueSecondIndex), mesh, meshHalfEdges, indexToEdgeMap);

                //-----------------instantiate second half edge---------------------//
                auto secondEdge = InstantiateEdge(secondVertIndex
                    , edgeVertexIndexPair(uniqueSecondIndex, uniqueThirdIndex), mesh, meshHalfEdges, indexToEdgeMap);

                //-----------------instantiate third half edge---------------------//
                auto thirdEdge = InstantiateEdge(thirdVertIndex
                    , edgeVertexIndexPair(uniqueThirdIndex, uniqueFirstIndex), mesh, meshHalfEdges, indexToEdgeMap);


                firstEdge->nextEdge = secondEdge;
                secondEdge->nextEdge = thirdEdge;
                thirdEdge->nextEdge = firstEdge;


            }

            //[3] connect each edge with its pair based on index
            log::debug("Created {} edges ", meshHalfEdges.size());

            for (auto indexEdgePair : indexToEdgeMap)
            {

                int u = indexEdgePair.first.first;
                int v = indexEdgePair.first.second;

                //for a Halfedge paired with vertices with an index of (u,v),
                //its pair would be a HalfEdge paired with vertices with an index of (v,u)
                auto iter = indexToEdgeMap.find(edgeVertexIndexPair(v, u));

                if (iter != indexToEdgeMap.end())
                {

                    auto  edgePair = iter->second;
                    auto  otherEdge = indexEdgePair.second;

                    edgePair->pairingEdge = otherEdge;
                    otherEdge->pairingEdge = edgePair;

                }
            }

            currentPtr = meshHalfEdges.front();

        }

        /** @brief Instantiate a new half edge and returns it if its unique
        */
        meshHalfEdgePtr InstantiateEdge(int vertexIndex
            , const std::pair<int, int> uniqueIndexPair
            , const mesh& mesh
            , std::queue<meshHalfEdgePtr>& edgePtrs
            , VertexIndexToHalfEdgePtr& indexToEdgeMap)
        {
            auto firstEdge = std::make_shared<MeshHalfEdge>(mesh.vertices[vertexIndex], mesh.uvs[vertexIndex]);

            auto edgeToAdd = UniqueAdd(firstEdge, indexToEdgeMap, uniqueIndexPair);

            edgePtrs.push(edgeToAdd);

            return edgeToAdd;
        }

        /** @brief Checks if a certain edge identified with 'uniqueIndexPair' is unique. If it is, it will
        * be added to 'edgePtrs'. If its not, the original will take its place.
        * @return a unique meshHalfEdgePtr
        */
        meshHalfEdgePtr UniqueAdd(meshHalfEdgePtr newEdge, VertexIndexToHalfEdgePtr& indexToEdgeMap, edgeVertexIndexPair uniqueIndexPair)
        {
            auto iter = indexToEdgeMap.find(uniqueIndexPair);

            if (iter != indexToEdgeMap.end())
            {
                return iter->second;
            }

            auto pair = uniqueIndexPair;
            indexToEdgeMap.insert({ pair ,newEdge });

            //vertexIndexToHalfEdge.Add(key, value);
            return newEdge;
        }

	};
}
