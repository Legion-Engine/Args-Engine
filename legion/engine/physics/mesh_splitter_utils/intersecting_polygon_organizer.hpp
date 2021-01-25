#pragma once
#include <core/core.hpp>
#include <physics/mesh_splitter_utils/splittable_polygon.hpp>
#include <physics/mesh_splitter_utils/mesh_splitter_typedefs.hpp>
#include <physics/mesh_splitter_utils/mesh_splitter_debug_helpers.hpp>
#include <physics/mesh_splitter_utils/intersection_edge_info.hpp>

namespace legion::physics
{
    /** @struct IntersectingPolygonOrganizer
    * @brief Contains the necessary operations to split the edges of a given polygon
    */
    struct IntersectingPolygonOrganizer
    {

        IntersectingPolygonOrganizer()
        {

        }

        //------------------------------------------------------ Related to Edge Categorization  ----------------------------------------------------------------//
        void SplitPolygon(SplittablePolygonPtr splitPolygon
            , const math::mat4& transform, const math::vec3 cutPosition
            , const math::vec3 cutNormal, SplitState requestedState
            , std::vector<IntersectionEdgeInfo>& generatedIntersectionEdges,bool shouldeDebug)
        {
            BoundaryEdgeInfo polygonDebugInfo;

            /*if (shouldeDebug)
            {
                DebugBreak();
            }*/
   

            bool keepAbove = requestedState == SplitState::Above ? true : false;
            //For now assume no islands in one polygon

            splitPolygon->ResetEdgeVisited();

            if (splitPolygon->GetMeshEdges().size() < 3) { return; }

            //---------------------------- [1] Get Triangles Effected by the split------------------------------------------------------------------//

            std::vector<meshHalfEdgePtr> unEffectedUsedEdges;
            std::vector<meshHalfEdgePtr> effectedUsedEdges;



            BFSIdentifyEffectedEdges(transform, splitPolygon, unEffectedUsedEdges, effectedUsedEdges, cutPosition, cutNormal, keepAbove);


            //---------------------------- [2] Get used boundary Effected ----------------------------------------------------------------------------//

            //set 'effectedUsedEdges' to be unvisited in order to differentiate it from 'unEffectedUsedEdges'
            for (auto edge : effectedUsedEdges)
            {
                edge->isVisited = false;
                
               

            }


            if (shouldeDebug)
            {
                log::debug("/// EDGE COUNT ///");
                log::debug(" splitPolygon->GetMeshEdges() {} ", splitPolygon->GetMeshEdges().size());
                log::debug(" unEffectedUsedEdges {} ", unEffectedUsedEdges.size());
                log::debug("effectedUsedEdges  {} ", effectedUsedEdges.size());

            }

            IdentifyBoundaryEffectedEdges(effectedUsedEdges,
                transform, cutPosition
                , cutNormal, keepAbove);

        
            for (auto edge : effectedUsedEdges)
            {
                if (shouldeDebug)
                {
                    math::color color = edge->isBoundary ? math::colors::blue : math::colors::cyan;
                    auto [first, second] = edge->getEdgeWorldPositions(transform);
                    //debug::user_projectDrawLine(first, second, color, 15.0f, FLT_MAX, true);
                }
            }


            //----------------------------- [3] Sort Edges ---------------------------------------------------------------------------------------//

            //find edges in effectedUsedEdges that are split by plane and store them in a std::vector
            std::vector<meshHalfEdgePtr> splitEdges;

            /*if (shouldeDebug)
            {
                DebugBreak();
                log::debug("----// should debug //---");
            }*/

            for (auto edge : effectedUsedEdges)
            {
                if (edge->isSplitByPlane(transform, cutPosition, cutNormal))
                {
                    splitEdges.push_back(edge);
                }
            }

            for (auto edge : splitEdges)
            {
                if (shouldeDebug)
                {
                    math::color color = math::colors::blue;
                    auto [first, second] = edge->getEdgeWorldPositions(transform);
                    //debug::user_projectDrawLine(first, second, color, 15.0f, FLT_MAX, true);
                }
            }

            //sort them based on support point created by cross product of normal and split normal

            math::vec3 worldCentroid = transform * math::vec4(splitPolygon->localCentroid, 1);
            math::vec3 worldPolygonNormal = transform * math::vec4(splitPolygon->localNormal, 0);
            polygonDebugInfo.worldNormal = worldPolygonNormal;

            math::vec3 polygonNormalCrossCutNormal = math::normalize(math::cross(worldPolygonNormal, cutNormal));

            //SortingCriterium splittingPlaneSorter(worldCentroid,polygonNormalCrossCutNormal,transform);


            auto initialSorter = [&transform, &worldCentroid, &polygonNormalCrossCutNormal]
            (const std::shared_ptr<MeshHalfEdge>& lhs, std::shared_ptr<MeshHalfEdge>& rhs)
            {
                math::vec3 aWorldCentroid = lhs->getWorldCentroid(transform);
                math::vec3 bWorldCentroid = rhs->getWorldCentroid(transform);

                math::vec3 AtoPolygonCentroid = aWorldCentroid - worldCentroid;
                math::vec3 BtoPolygonCentroid = bWorldCentroid - worldCentroid;

                return
                    math::dot(AtoPolygonCentroid, polygonNormalCrossCutNormal) <
                    math::dot(BtoPolygonCentroid, polygonNormalCrossCutNormal);
            };

            std::sort(splitEdges.begin(), splitEdges.end(), initialSorter);

            //clear the polygon's edge list because we are going to start regenerating it
            auto& meshEdges = splitPolygon->GetMeshEdges();
            meshEdges.clear();

            int edgesUsed = effectedUsedEdges.size();
            static int numberOfEdgesToMakeATriangle = 2;

            //Check for the special case where there are only 2 splitEdges, this will create 1 triangle
            if (edgesUsed == numberOfEdgesToMakeATriangle)
            {
                HandleTriangleSplit(splitPolygon,
                    splitEdges, generatedIntersectionEdges,
                    transform, cutNormal, cutPosition, keepAbove);

                return;
            }
            else if(edgesUsed < numberOfEdgesToMakeATriangle)
            {
                //there exist a number of edges that is on the splitting plane
                //assert(false);
                return;
            }

            auto firstSplit = splitEdges.at(0);
            auto secondSplit = splitEdges.at(splitEdges.size() - 1);

            int firstIndex = -1;
            int secondIndex = -1;
            for (size_t i = 0; i < effectedUsedEdges.size(); i++)
            {
                if (effectedUsedEdges.at(i) == firstSplit)
                {
                    firstIndex = i;
                }

                if (effectedUsedEdges.at(i) == secondSplit)
                {
                    secondIndex = i;
                }
            }
            if (shouldeDebug)
            {
                log::debug("firstIndex {} secondIndex {} ", firstIndex, secondIndex);


            }
           
            //I am aware of how horrible this is but I really want to get this done
            int offset = firstIndex < secondIndex ? -1 : 0;
            effectedUsedEdges.erase(effectedUsedEdges.begin() + firstIndex);
            effectedUsedEdges.erase(effectedUsedEdges.begin() + math::clamp(secondIndex + offset,0,999));
            //previous sorting direction is dependent on cut tangent.
            //use centroids of first and last split edge as the new sorting direction

            math::vec3 worldFirstEdge = firstSplit->getWorldCentroid(transform);
            math::vec3 worldSecondEdge = secondSplit->getWorldCentroid(transform);

            //sort effectedUsedEdges based on sorting direction

            math::vec3 sortingCentroid = (worldFirstEdge + worldSecondEdge) / 2.0f;
            math::vec3 sortingDirection = worldSecondEdge - worldFirstEdge;

            auto boundarySorter = [&transform, &sortingCentroid, &sortingDirection]
            (const std::shared_ptr<MeshHalfEdge>& lhs, std::shared_ptr<MeshHalfEdge>& rhs)
            {
                math::vec3 aWorldCentroid = lhs->getWorldCentroid(transform);
                math::vec3 bWorldCentroid = rhs->getWorldCentroid(transform);

                math::vec3 AtoPolygonCentroid = aWorldCentroid - sortingCentroid;
                math::vec3 BtoPolygonCentroid = bWorldCentroid - sortingCentroid;

                return
                    math::dot(AtoPolygonCentroid, sortingDirection) <
                    math::dot(BtoPolygonCentroid, sortingDirection);
            };

            std::sort(effectedUsedEdges.begin(), effectedUsedEdges.end(), boundarySorter);
            effectedUsedEdges.insert(effectedUsedEdges.begin(), firstSplit);
            effectedUsedEdges.push_back(secondSplit);

            //===================DEBUG
            if (shouldeDebug)
            {
                float max = effectedUsedEdges.size();
                int i = 0;

                log::debug("effectedUsedEdges {} ", effectedUsedEdges.size());

                for (auto meshHalfEdgePtr : effectedUsedEdges)
                {
                    auto [first,second]=  meshHalfEdgePtr->getEdgeWorldPositions(transform);
                    float interpolant = (float)i / max;
                    math::vec3 white = math::vec3(1, 0, 0) * interpolant;
                    //debug::user_projectDrawLine(first, second, math::color(white.x, white.y, white.z, 1), 12.0f, FLT_MAX, true);
                    
                    i++;
                }

               /* debug::user_projectDrawLine(sortingCentroid, sortingCentroid + sortingDirection
                    , math::colors::cyan, 12.0f, FLT_MAX, true);*/

            }
      
            //------------------------------------- [5] Regenerate Edges ----------------------------------------------------------------------------------------------------------//

            //get start and end intersection points
            auto [firstEdgeCurrent, firstEdgeNext] = firstSplit->getEdgeWorldPositions(transform);

            //check if position of first edge is outside intersection, this is important because it will determine how the edges will be regenerated
            bool startFromOutsideIntersection =
                keepAbove == PhysicsStatics::IsPointAbovePlane(cutNormal, cutPosition, firstEdgeCurrent);

            auto [firstEdgeIntersection, firstInterpolantUV, secondEdgeIntersection, secondInterpolantUV]
                = GetFirstAndLastEdgeIntersectionInfo(cutNormal, cutPosition, transform,
                firstSplit, secondSplit);

            math::vec3 startToEndIntersection = secondEdgeIntersection - firstEdgeIntersection;
            math::vec2 startToEndUV = secondInterpolantUV - firstInterpolantUV;

            meshHalfEdgePtr supportEdge = nullptr;
            //generated half edges using
            std::vector<meshHalfEdgePtr> generatedHalfEdges;

            SplitterIntersectionInfo vertexIntersectionInfo(
                firstEdgeIntersection,
                startToEndIntersection,
                firstInterpolantUV,
                startToEndUV);

            for (int i = 1; i < effectedUsedEdges.size() - 1; i++)
            {
                if (startFromOutsideIntersection)
                {
                    InsideIntersectionMeshRegeneration(
                        transform, effectedUsedEdges,
                        generatedHalfEdges, i, supportEdge, vertexIntersectionInfo, polygonDebugInfo, generatedIntersectionEdges,splitPolygon);
                }
                else
                {
                    OutsideIntersectionMeshRegeneration(
                        transform, effectedUsedEdges,
                        generatedHalfEdges, i, supportEdge, vertexIntersectionInfo, polygonDebugInfo, generatedIntersectionEdges, splitPolygon);
                }
            }

            polygonDebugInfo.intersectionPoints.first = firstEdgeIntersection;
            polygonDebugInfo.intersectionPoints.second = secondEdgeIntersection;

            polygonDebugInfo.finalSortingDirection.first = sortingCentroid;
            polygonDebugInfo.finalSortingDirection.second = sortingDirection;

            polygonDebugInfo.drawColor = splitPolygon->debugColor;

            //--------------DEBUG STUFF----------------//
            polygonDebugInfo.boundaryEdges.insert(
                polygonDebugInfo.boundaryEdges.begin(),
                effectedUsedEdges.begin(),
                effectedUsedEdges.end());

            polygonDebugInfo.boundaryEdges.insert(
                polygonDebugInfo.boundaryEdges.end(),
                generatedHalfEdges.begin(),
                generatedHalfEdges.end());
            //-------------------------------------//

            //debugHelper->boundaryEdgesForPolygon.push_back(polygonDebugInfo);

            //------------------------------------- [6] Add regenerated edges to the polygon -----------------------------------------------------------------------------------//

            meshEdges.insert(meshEdges.end(), unEffectedUsedEdges.begin(), unEffectedUsedEdges.end());
            meshEdges.insert(meshEdges.end(), effectedUsedEdges.begin(), effectedUsedEdges.end());
            meshEdges.insert(meshEdges.end(), generatedHalfEdges.begin(), generatedHalfEdges.end());

            splitPolygon->CalculateLocalCentroid();
            splitPolygon->AssignEdgeOwnership();
            //debugHelper->polygonCount++;
        }

        //------------------------------------------------------ Edge Categorization Helper Functions ----------------------------------------------------------------//

        std::tuple<math::vec3, math::vec2, math::vec3, math::vec2> GetFirstAndLastEdgeIntersectionInfo(const math::vec3& cutNormal, const math::vec3& cutPosition
            , const math::mat4 transform,
            meshHalfEdgePtr firstEdge, meshHalfEdgePtr secondEdge)
        {
            auto [firstEdgeCurrent, firstEdgeNext] = firstEdge->getEdgeWorldPositions(transform);
            auto [secondEdgeCurrent, secondEdgeNext] = secondEdge->getEdgeWorldPositions(transform);

            math::vec3 firstEdgeIntersection;
            float firstEdgeInterpolant;
            PhysicsStatics::FindLineToPlaneIntersectionPoint(cutNormal, cutPosition,
                firstEdgeCurrent, firstEdgeNext, firstEdgeIntersection, firstEdgeInterpolant);

            math::vec2 firstInterpolantUV = math::lerp(firstEdge->uv, firstEdge->nextEdge->uv, firstEdgeInterpolant);

            math::vec3 secondEdgeIntersection;
            float secondEdgeInterpolant;
            PhysicsStatics::FindLineToPlaneIntersectionPoint(cutNormal, cutPosition,
                secondEdgeCurrent, secondEdgeNext, secondEdgeIntersection, secondEdgeInterpolant);

            math::vec2 secondInterpolantUV = math::lerp(secondEdge->uv, secondEdge->nextEdge->uv, secondEdgeInterpolant);

            math::vec3 startToEndIntersection = secondEdgeIntersection - firstEdgeIntersection;
            math::vec2 startToEndUV = secondInterpolantUV - firstInterpolantUV;

            return std::make_tuple(firstEdgeIntersection, firstInterpolantUV, secondEdgeIntersection, secondInterpolantUV);
        }

        void RegenerateTriangleSplit(bool startFromOutsideIntersection
            , meshHalfEdgePtr firstSplitEdge, meshHalfEdgePtr secondSplitEdge
            , meshHalfEdgePtr& intersectionEdge, const math::vec3& cutNormal, const math::vec3& cutPosition,
            const math::mat4& transform)
        {
            auto [firstEdgeIntersection, firstInterpolantUV, secondEdgeIntersection, secondInterpolantUV]
                = GetFirstAndLastEdgeIntersectionInfo(cutNormal, cutPosition, transform,
                    firstSplitEdge, secondSplitEdge);

            math::vec3 startToEndIntersection = secondEdgeIntersection - firstEdgeIntersection;
            math::vec2 startToEndUV = secondInterpolantUV - firstInterpolantUV;

            const math::mat4 inverseTrans{ math::inverse(transform) };

            if (!startFromOutsideIntersection)
            {
                firstSplitEdge->position = inverseTrans * math::vec4(firstEdgeIntersection, 1);
                firstSplitEdge->uv = firstInterpolantUV;

                intersectionEdge = std::make_shared<MeshHalfEdge>
                    (inverseTrans * math::vec4(secondEdgeIntersection, 1), secondInterpolantUV);
                

                MeshHalfEdge::connectIntoTriangle(firstSplitEdge, secondSplitEdge, intersectionEdge);
            }
            else
            {
                intersectionEdge = std::make_shared<MeshHalfEdge>
                    (inverseTrans * math::vec4(firstEdgeIntersection, 1), firstInterpolantUV);

                secondSplitEdge->position = inverseTrans * math::vec4(secondEdgeIntersection, 1);
                secondSplitEdge->uv = secondInterpolantUV;

                MeshHalfEdge::connectIntoTriangle(firstSplitEdge, intersectionEdge, secondSplitEdge);
            }

            intersectionEdge->isBoundary = true;
        }

        void BFSIdentifyEffectedEdges(const math::mat4& transform, SplittablePolygonPtr splitPolygon,
            std::vector<meshHalfEdgePtr>& unEffectedUsedEdges
            , std::vector<meshHalfEdgePtr>& effectedUsedEdges
            , const math::vec3 cutPosition,const math::vec3 cutNormal,bool keepAbove)
        {
            meshHalfEdgePtr initalEdge = splitPolygon->GetMeshEdges().at(0);
            //intialize unvisited queue
            std::queue<meshHalfEdgePtr> unvisitedEdges;
            unvisitedEdges.push(initalEdge);

            while (!unvisitedEdges.empty())
            {
                auto currentEdge = unvisitedEdges.front();
                unvisitedEdges.pop();

                if (!currentEdge->isVisited)
                {
                    currentEdge->markTriangleEdgeVisited();

                    auto [edge1, edge2, edge3] = currentEdge->getTriangle();

                    bool edge1Split = edge1->isSplitByPlane(transform, cutPosition, cutNormal);
                    bool edge2Split = edge2->isSplitByPlane(transform, cutPosition, cutNormal);
                    bool edge3Split = edge3->isSplitByPlane(transform, cutPosition, cutNormal);

                    if (edge1Split || edge2Split || edge3Split)
                    {
                        currentEdge->populateVectorWithTriangle(effectedUsedEdges);
                    }
                    else
                    {
                        //if we reach this point the triangle is either completely above the plane or completly below it
                        //we cache it only if its at the correct position
                        math::vec3 edgePosition = transform * math::vec4(currentEdge->position, 1);

                        if (keepAbove == PhysicsStatics::IsPointAbovePlane(cutNormal, cutPosition, edgePosition))
                        {
                            currentEdge->populateVectorWithTriangle(unEffectedUsedEdges);
                        }

                    }

                    if (!edge1->isBoundary)
                    {
                        unvisitedEdges.push(edge1->pairingEdge);
                    }

                    if (!edge2->isBoundary)
                    {
                        unvisitedEdges.push(edge2->pairingEdge);
                    }

                    if (!edge3->isBoundary)
                    {
                        unvisitedEdges.push(edge3->pairingEdge);
                    }
                }
            }

        }

        void IdentifyBoundaryEffectedEdges( std::vector<meshHalfEdgePtr>& effectedUsedEdges,
            const math::mat4 transform, const math::vec3 cutPosition
            , const math::vec3 cutNormal, bool keepAbove)
        {
            std::vector<meshHalfEdgePtr> tempEdges{ std::move(effectedUsedEdges) };
            effectedUsedEdges.clear();

            for (auto edge : tempEdges)
            {
                bool isAtLeastPartiallyAtRequestedSpot = keepAbove ?
                    edge->isEdgePartlyAbovePlane(transform, cutPosition, cutNormal)
                    : edge->isEdgePartlyBelowPlane(transform, cutPosition, cutNormal);

                bool isBoundaryEffectedOrPolygonBoundary = edge->isBoundary || edge->pairingEdge->isVisited;

                if (isAtLeastPartiallyAtRequestedSpot && isBoundaryEffectedOrPolygonBoundary)
                {
                    effectedUsedEdges.push_back(std::move(edge));
                }

            }

            //tempEdges gets destroyed at end of scope
        }

        void HandleTriangleSplit(SplittablePolygonPtr splitPolygon,
            std::vector<meshHalfEdgePtr>& splitEdges, std::vector<IntersectionEdgeInfo>& generatedIntersectionEdges,
            const math::mat4& transform, const math::vec3& cutNormal,const math::vec3& cutPosition, bool keepAbove)
        {
            auto& meshEdges = splitPolygon->GetMeshEdges();

            meshHalfEdgePtr firstEdge = splitEdges.at(0);
            meshHalfEdgePtr secondEdge = splitEdges.at(1);

            math::vec3 edgeWorldPosition = firstEdge->getEdgeWorldPosition(transform);

            bool startFromOutsideIntersection =
                keepAbove == PhysicsStatics::IsPointAbovePlane(cutNormal, cutPosition, edgeWorldPosition);

            //handle triangle split
            meshHalfEdgePtr intersectionEdge = nullptr;

            RegenerateTriangleSplit(startFromOutsideIntersection
                , firstEdge, secondEdge,
                intersectionEdge, cutNormal, cutPosition,
                transform);

            IntersectionEdgeInfo intersectionEdgeInfo(intersectionEdge);

            generatedIntersectionEdges.push_back(intersectionEdgeInfo);


            meshEdges.push_back(firstEdge);
            meshEdges.push_back(secondEdge);
            meshEdges.push_back(intersectionEdge);

            splitPolygon->CalculateLocalCentroid();
            splitPolygon->AssignEdgeOwnership();
        }


        //------------------------------------------------------ Related to Quad Organization ----------------------------------------------------------------//

        void InsideIntersectionMeshRegeneration(const math::mat4& transform, std::vector<meshHalfEdgePtr>& effectedBoundaryEdges,
            std::vector<meshHalfEdgePtr>& generatedEdges, int i
            , meshHalfEdgePtr& supportEdge,
            const SplitterIntersectionInfo& vertexIntersectionInfo, BoundaryEdgeInfo& debugStuff, std::vector<IntersectionEdgeInfo>& generatedIntersectionEdges
            , SplittablePolygonPtr owner)
        {
            //select Base Half Edge
            meshHalfEdgePtr baseEdge = effectedBoundaryEdges[i];
            debugStuff.base = baseEdge->getEdgeWorldPosition(transform);


            /*const math::vec3& initialIntersectionPosition = vertexIntersectionInfo.startIntersectionPosition;
            const math::vec3& startToEndPosition = vertexIntersectionInfo.startToEndPosition;
            const math::vec2& initialUVIntersection = vertexIntersectionInfo.startIntersectionUV;
            const math::vec2& startToEndUV = vertexIntersectionInfo.startToEndUV;*/

            auto [initialIntersectionPosition, startToEndPosition,
                initialUVIntersection, startToEndUV] = vertexIntersectionInfo.GetIntersectionData();

            //----------------------------------- Create currentSupportEdge -------------------------------------------------------------//
            meshHalfEdgePtr currentSupportEdge = nullptr;

            if (i == 1)
            {
                currentSupportEdge = effectedBoundaryEdges[0];
                //currentSupportEdge.position = transform.worldToLocalMatrix.MultiplyPoint(worldStartIntersection);
            }
            else
            {
                currentSupportEdge = std::make_shared<MeshHalfEdge>
                    (baseEdge->nextEdge->position, baseEdge->nextEdge->uv, owner);
               

                currentSupportEdge->setPairing(supportEdge);
                generatedEdges.push_back(currentSupportEdge);

            }

            debugStuff.prevSupport = currentSupportEdge->getEdgeWorldPosition(transform);
            //----------------------------------- Create nextcurrentSupportEdge -------------------------------------------------------------//
            int maxData = effectedBoundaryEdges.size() - 1;
            meshHalfEdgePtr nextSupportEdge;

            if (i + 1 == effectedBoundaryEdges.size() - 1)
            {
                nextSupportEdge = effectedBoundaryEdges[effectedBoundaryEdges.size() - 1];

                math::vec3 worldPosition =
                    (initialIntersectionPosition + startToEndPosition);

                nextSupportEdge->position = math::inverse(transform) * math::vec4((worldPosition), 1);
                nextSupportEdge->uv = initialUVIntersection + startToEndUV;
            }
            else
            {
                math::vec3 worldPosition = initialIntersectionPosition
                    + startToEndPosition * (float)i / maxData;

                math::vec2 uv = initialUVIntersection + startToEndUV * (float)i / maxData;

                nextSupportEdge = std::make_shared< MeshHalfEdge>(
                    math::inverse(transform) * math::vec4(worldPosition, 1), uv, owner);

                generatedEdges.push_back(nextSupportEdge);
            }

            debugStuff.nextSupport = nextSupportEdge->getEdgeWorldPosition(transform);

            //----------------------------------- Create Intersection -------------------------------------------------------------//

            int currentIndex = i - 1;


            //i - 1 < 0 ? 0 : i - 1 ;
            float interpolant = (float)currentIndex / maxData;

            math::vec3 worldIntersectionPosition =
                initialIntersectionPosition + (startToEndPosition * interpolant);

            math::vec3 localIntersectionPosition = math::inverse(transform)
                * math::vec4(worldIntersectionPosition, 1);

            math::vec2 uv = initialUVIntersection + startToEndUV * interpolant;

            meshHalfEdgePtr intersectionEdge = std::make_shared< MeshHalfEdge>(
                localIntersectionPosition, uv, owner);


            generatedEdges.push_back(intersectionEdge);
            intersectionEdge->isBoundary = true;

            debugStuff.intersectionEdge = intersectionEdge->getEdgeWorldPosition(transform);

            CreateNonAllignedQuad(currentSupportEdge, nextSupportEdge, baseEdge, intersectionEdge, generatedEdges,owner);

            //-------------------------- Insantiate intersection info ----------------------------------------------//

            IntersectionEdgeInfo intersectionEdgeInfo(
                intersectionEdge);
            generatedIntersectionEdges.push_back(intersectionEdgeInfo);
            

            supportEdge = nextSupportEdge;



        }

        void CreateNonAllignedQuad(
            meshHalfEdgePtr currentSupport, meshHalfEdgePtr nextSupport, meshHalfEdgePtr baseEdge, meshHalfEdgePtr intersectionEdge,
            std::vector<meshHalfEdgePtr>& generatedEdges, SplittablePolygonPtr owner
        )
        {
            //create new supporttriangle located at next support
            meshHalfEdgePtr supportTriangle = std::make_shared< MeshHalfEdge>(nextSupport->position, nextSupport->uv, owner);


            //currentSupport-intersection-supporttriangle
            MeshHalfEdge::connectIntoTriangle(currentSupport, intersectionEdge, supportTriangle);

            //create new nextsupporttriangle located at currentsupport
            meshHalfEdgePtr nextSupportTriangle = std::make_shared< MeshHalfEdge>(currentSupport->position, currentSupport->uv, owner);

            //nextsupporttriangle-nextSupport-baseEdge
            MeshHalfEdge::connectIntoTriangle(nextSupportTriangle, nextSupport, baseEdge);

            supportTriangle->setPairing(nextSupportTriangle);
            generatedEdges.push_back(supportTriangle);
            generatedEdges.push_back(nextSupportTriangle);


        }

        void OutsideIntersectionMeshRegeneration(const math::mat4& transform, std::vector<meshHalfEdgePtr>& effectedBoundaryEdges,
            std::vector<meshHalfEdgePtr>& generatedEdges, int i
            , meshHalfEdgePtr& supportEdge,
            const SplitterIntersectionInfo& vertexIntersectionInfo, BoundaryEdgeInfo& debugStuff, std::vector<IntersectionEdgeInfo>& generatedIntersectionEdges
            , SplittablePolygonPtr owner)
        {
            meshHalfEdgePtr baseEdge = effectedBoundaryEdges[i];

            auto [initialIntersectionPosition, startToEndPosition,
                initialUVIntersection, startToEndUV] = vertexIntersectionInfo.GetIntersectionData();

            //----------------------------------- Create currentSupportEdge -------------------------------------------------------------//
            meshHalfEdgePtr currentSupportEdge;
            if (i == 1)
            {
                currentSupportEdge = effectedBoundaryEdges[0];

                currentSupportEdge->position = math::inverse(transform) *
                    math::vec4(initialIntersectionPosition, 1);

                currentSupportEdge->uv = initialUVIntersection;

            }
            else
            {
                //assert(supportEdge->nextEdge);
                currentSupportEdge = std::make_shared<MeshHalfEdge>
                    (supportEdge->nextEdge->position, supportEdge->nextEdge->uv, owner);

                currentSupportEdge->setPairing(supportEdge);
                generatedEdges.push_back(currentSupportEdge);
            }


            //----------------------------------- Create nextcurrentSupportEdge -------------------------------------------------------------//
            meshHalfEdgePtr  nextSupportEdge = nullptr;
            if (i + 1 == effectedBoundaryEdges.size() - 1)
            {
                nextSupportEdge = effectedBoundaryEdges[effectedBoundaryEdges.size() - 1];
                //currentSupportEdge.position = transform.worldToLocalMatrix.MultiplyPoint(worldStartIntersection + startToEndIntersection);
            }
            else
            {
                nextSupportEdge = std::make_shared<MeshHalfEdge>(baseEdge->nextEdge->position, baseEdge->nextEdge->uv, owner);
                generatedEdges.push_back(nextSupportEdge);
            }

            supportEdge = nextSupportEdge;
            //----------------------------------- Create Intersection -------------------------------------------------------------//
            //Vector3 intersectionPosition;
            int maxData = effectedBoundaryEdges.size() - 1;
            int currentIndex = i + 1;
            float interpolant = (float)currentIndex / maxData;

            math::vec3 IntersectionEdgePosition =
                initialIntersectionPosition + (startToEndPosition * interpolant);

            math::vec3 localIntersectionEdgePosition =
                math::inverse(transform) * math::vec4(IntersectionEdgePosition, 1);

            math::vec2 edgeUV = initialUVIntersection + startToEndUV * interpolant;

            meshHalfEdgePtr intersectionEdge = std::make_shared<MeshHalfEdge>
                (localIntersectionEdgePosition, edgeUV, owner);
            intersectionEdge->isBoundary = true;
            generatedEdges.push_back(intersectionEdge);

            CreateAllignedQuad(currentSupportEdge, nextSupportEdge, baseEdge,
                intersectionEdge, generatedEdges, owner);

            IntersectionEdgeInfo intersectionEdgeInfo(
                intersectionEdge);

            generatedIntersectionEdges.push_back(intersectionEdgeInfo);

            

        }

        void CreateAllignedQuad(meshHalfEdgePtr currentSupport, meshHalfEdgePtr nextSupport, meshHalfEdgePtr baseEdge,
            meshHalfEdgePtr intersectionEdge, std::vector<meshHalfEdgePtr>& generatedEdges, SplittablePolygonPtr owner)
        {
            //create new supporttriangle located at next support
            meshHalfEdgePtr supportTriangle = std::make_shared<MeshHalfEdge>(nextSupport->position, nextSupport->uv, owner);

            //currentSupport-intersection-supporttriangle
            MeshHalfEdge::connectIntoTriangle(currentSupport, supportTriangle, intersectionEdge);

            //create new nextsupporttriangle located at currentsupport
            meshHalfEdgePtr  nextSupportTriangle = std::make_shared<MeshHalfEdge>(currentSupport->position, currentSupport->uv, owner);

            //nextsupporttriangle-nextSupport-baseEdge
            MeshHalfEdge::connectIntoTriangle(nextSupportTriangle, baseEdge, nextSupport);

            supportTriangle->setPairing(nextSupportTriangle);

            generatedEdges.push_back(supportTriangle);
            generatedEdges.push_back(nextSupportTriangle);
        }

    };





}
