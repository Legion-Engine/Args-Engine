#include <physics/mesh_splitter_utils/mesh_splitter.hpp>

namespace legion::physics
{
    void MeshSplitter::InitializePolygons(ecs::entity_handle entity)
    {
        owner = entity;

        auto [meshFilter, meshRenderer] = entity.get_component_handles<rendering::mesh_renderable>();

        ownerMaterialH = meshRenderer.read().material;

        auto [posH, rotH, scaleH] = entity.get_component_handles<transform>();

        if (meshFilter && posH && rotH && scaleH)
        {
            log::debug("Mesh and Transform found");
            std::queue<meshHalfEdgePtr> meshHalfEdges;

            //auto renderable = renderable.read();
            mesh& mesh = meshFilter.read().get().second;

            const math::mat4 transform = math::compose(scaleH.read(), rotH.read(), posH.read());
            //debugHelper.DEBUG_transform = transform;
          
            HalfEdgeFinder edgeFinder;
            edgeFinder.FindHalfEdge(mesh, transform, meshHalfEdges);

            BFSPolygonize(meshHalfEdges, transform);

            log::debug("Mesh vertices {}, Mesh indices {}", mesh.vertices.size(), mesh.indices.size());

        }
        else
        {
            log::warn("The given entity does not have a meshHandle!");
        }

        for (auto face : meshPolygons)
        {
            for (auto edge : face->GetMeshEdges())
            {
                assert(edge->owner.lock());
                assert(edge->pairingEdge->owner.lock());
            }
        }



    }

    void MeshSplitter::MultipleSplitMesh(const std::vector<MeshSplitParams>& splittingPlanes,
        std::vector<ecs::entity_handle>& entitiesGenerated, bool keepBelow, int debugAt)
    {
        int currentDebug = 0;

        auto [posH, rotH, scaleH] = owner.get_component_handles<transform>();
        const math::mat4& transform = math::compose(scaleH.read(), rotH.read(), posH.read());

        //-------------------------------- copy polygons of original mesh and add it to the output list -----------------------------------------//

        std::vector< std::vector<SplittablePolygonPtr>> outputPolygonIslandsGenerated;

        std::vector<SplittablePolygonPtr> copiedPolygons;
        CopyPolygons(meshPolygons, copiedPolygons);

        outputPolygonIslandsGenerated.push_back(std::move(copiedPolygons));

        //-------------------------------- spllit mesh based on list of splitting planes -----------------------------------------//
        for (const MeshSplitParams& splitParam : splittingPlanes)
        {
            std::vector< std::vector<SplittablePolygonPtr>> inputList = std::move(outputPolygonIslandsGenerated);
            outputPolygonIslandsGenerated.clear();

            for (std::vector<SplittablePolygonPtr>& polygonIsland : inputList)
            {
                for (SplittablePolygonPtr polygon : polygonIsland)
                {
                    polygon->isVisited = false;
                }

                SplitPolygons
                (polygonIsland,
                    splitParam.planeNormal,
                    splitParam.planePostion,
                    transform,
                    outputPolygonIslandsGenerated, keepBelow,currentDebug == debugAt);
            }
            currentDebug++;
        }

        //-------------------------------- use each polygon list to create a new object -----------------------------------------//

        for (auto& polygonIsland : outputPolygonIslandsGenerated)
        {
            PrimitiveMesh newMesh(owner, polygonIsland, ownerMaterialH);
            auto newEnt = newMesh.InstantiateNewGameObject();

            entitiesGenerated.push_back(newEnt);
        }
    }

    void MeshSplitter::SplitPolygons(std::vector<SplittablePolygonPtr>& polygonsToSplit, const math::vec3& planeNormal, const math::vec3& planePosition,
        const math::mat4& transform, std::vector<std::vector<SplittablePolygonPtr>>& resultingIslands, bool keepBelow, bool shouldDebug)
    {
    
        //log::debug("SplitPolygons");

        //----------------------- Find out which polygons are below,above, or intersecting the splitting plane -----------------------------------//

        for (auto polygon : polygonsToSplit)
        {
            //log::debug("Polygon has {} edges  ", polygon->GetMeshEdges().size());
            polygon->isVisited = false;
            polygon->CalculatePolygonSplit(transform, planePosition, planeNormal, keepBelow);

           /* switch (polygon->GetPolygonSplitState())
            {
            case SplitState::Above:
                log::debug("Above ");
                break;
            case SplitState::Below:
                log::debug("Below ");
                break;
            case SplitState::Split:
                log::debug("Split ");
                break;
            case SplitState::Unknown:
                log::debug("Unknown ");
                break;

            default:
                break;
            }*/

        }

        SplitState requestedState = keepBelow ? SplitState::Below : SplitState::Above;

        SplittablePolygonPtr initialFound = nullptr;

        bool foundUnvisited =
            FindFirstIntersectingOrRequestedState
            (initialFound, requestedState, polygonsToSplit);

        /*if (math::distance(planeNormal, math::vec3(-1, 0, 0)) < 0.01f)
        {
            DebugBreak();
        }*/
        //while there is an unvisited polygon that is on the requestedState or is intersecting the splitting plane
        while (foundUnvisited)
        {
            std::vector<SplittablePolygonPtr> splitMesh;
            std::vector<SplittablePolygonPtr> nonSplitMesh;

            //------------------------ BFS search polygons that are in the same island -------------------------------------------//
            //------------------------ and  divide them into a list of split and nonsplit polygons--------------------------//
            initialFound->isVisited = false;
            BFSFindRequestedAndIntersecting(
                initialFound,
                splitMesh,
                nonSplitMesh, requestedState);


            std::vector < std::vector <meshHalfEdgePtr>> holeIslands;
            std::vector<std::vector<SplittablePolygonPtr>> intersectionIslands;

            //----------------------------------- Detect multiple holes in mesh  --------------------------------------------------//

            DetectIntersectionIsland(splitMesh, intersectionIslands);

            //----------------------------------- Filter Edges in polygon in order to fit sliced mesh --------------------------------------------------//

            std::vector<IntersectionEdgeInfo> generatedIntersectionEdges;
            int indexCurrent = 0;
            int indexToSee = 1;

            /*if (shouldDebug)
            {
                DebugBreak();
            }*/

            for (std::vector<SplittablePolygonPtr>& intersectionIsland : intersectionIslands)
            {

                for (SplittablePolygonPtr islandPolygon : intersectionIsland)
                {
                    SplitPolygon(islandPolygon, transform, planePosition, planeNormal, requestedState, generatedIntersectionEdges, indexCurrent == indexToSee && shouldDebug);
              
                    if (indexCurrent == indexToSee && shouldDebug)
                    {
                        log::debug("indexCurrent {} ", indexCurrent);
                        log::debug("generatedIntersectionEdges {} ", generatedIntersectionEdges.size());
                        /*
                        math::vec3 first = debugHelper.DEBUG_transform * math::vec4(generatedIntersectionEdges.back().first, 1);
                        math::vec3 second = debugHelper.DEBUG_transform * math::vec4(generatedIntersectionEdges.back().second, 1);

                        debug::user_projectDrawLine(first, second, math::colors::magenta, 12.0f, FLT_MAX, true);*/
                    }
                    

                    indexCurrent++;
                }
            }
            

            math::vec3 localNormal = transform * math::vec4(planeNormal, 0);
            SplittablePolygonPtr intersectionPolygon = CreateIntersectionPolygon(generatedIntersectionEdges, math::normalize(localNormal));
            intersectionPolygon->isVisited = true;
            intersectionPolygon->ResetEdgeVisited();

            float max = generatedIntersectionEdges.size();
            int i = 0;
            for (auto intersectionInfo : generatedIntersectionEdges)
            {
               
                if (shouldDebug)
                {
                    float interpolant = (float)i / max;
                    math::vec3 color = math::color(1, 0, 1) * interpolant;
                    //log::debug("generatedIntersectionEdges {} ", generatedIntersectionEdges.size());
                   /* math::vec3 first = debugHelper.DEBUG_transform * math::vec4(intersectionInfo.first, 1);
                    math::vec3 second = debugHelper.DEBUG_transform * math::vec4(intersectionInfo.second, 1);*/

                    /*debug::user_projectDrawLine(first + (second-first) * 0.05f, second + (first - second) * 0.05f, math::color(color.x,color.y,color.z,1.0f),
                        math::linearRand(8.0f,12.0f), FLT_MAX, true);*/
                }

                i++;
            }

            //---------------------------------- Add intersecting and nonsplit to primitive mesh ---------------------------------//

            std::vector< SplittablePolygonPtr> resultPolygons;

            resultPolygons.insert(resultPolygons.end()
                , std::make_move_iterator(splitMesh.begin()), std::make_move_iterator(splitMesh.end()));

            resultPolygons.insert(resultPolygons.end()
                , std::make_move_iterator(nonSplitMesh.begin()), std::make_move_iterator(nonSplitMesh.end()));

            if (!generatedIntersectionEdges.empty())
            {
                resultPolygons.push_back(std::move(intersectionPolygon));
            }
 

            foundUnvisited =
                FindFirstIntersectingOrRequestedState
                (initialFound, requestedState, polygonsToSplit);

            resultingIslands.push_back(std::move(resultPolygons));

        }
    }

    void MeshSplitter::CopyPolygons(std::vector<SplittablePolygonPtr>& originalSplitMesh,
        std::vector<SplittablePolygonPtr>& copySplitMesh)
    {

        //----copy all edges of all polygons and connect them to each edge that is within the same polygon--//
        for (SplittablePolygonPtr originalPolygon : originalSplitMesh)
        {
            originalPolygon->ResetEdgeVisited();

            std::vector<meshHalfEdgePtr> copyPolygonEdges;
            CopyEdgeVector(originalPolygon->GetMeshEdges(), copyPolygonEdges);

            auto copyPolygon = std::make_shared<SplittablePolygon>(copyPolygonEdges, originalPolygon->localNormal);
            copyPolygon->AssignEdgeOwnership();
            copySplitMesh.push_back(copyPolygon);
        }

        //----use shadow edge to connect boundary edges between copied polygons--//

        for (SplittablePolygonPtr originalPolygon : originalSplitMesh)
        {


            for (auto originalEdge : originalPolygon->GetMeshEdges())
            {
                if (originalEdge->isBoundary && originalEdge->pairingEdge)
                {
                    auto shadowEdge = originalEdge->shadowEdge;
                    auto shadowEdgePairing = originalEdge->pairingEdge->shadowEdge;

                    shadowEdge->setPairing(shadowEdgePairing);
                }

            }
        }

        //----nullify shadow edge for each edge in the original polygon list--//

        for (SplittablePolygonPtr originalPolygon : originalSplitMesh)
        {
            originalPolygon->ResetEdgeVisited();

            for (auto originalEdge : originalPolygon->GetMeshEdges())
            {
                originalEdge->shadowEdge = nullptr;
            }
        }
    }

    void MeshSplitter::CopyEdgeVector(std::vector<meshHalfEdgePtr>& originalHalfEdgeList, std::vector<meshHalfEdgePtr>& resultCopyList)
    {
        for (meshHalfEdgePtr originalEdge : originalHalfEdgeList)
        {
            //copy polygon and place on shadow edge
            originalEdge->cloneOnShadowEdge();
        }

        //BFS connect with clone edge
        std::queue<meshHalfEdgePtr> unvisitedOriginalHalfEdgeList;
        unvisitedOriginalHalfEdgeList.push(originalHalfEdgeList[0]);

        while (!unvisitedOriginalHalfEdgeList.empty())
        {
            auto originalEdge = unvisitedOriginalHalfEdgeList.front();
            unvisitedOriginalHalfEdgeList.pop();

            if (!originalEdge->isVisited)
            {
                originalEdge->isVisited = true;

                auto [original1, original2, original3] = originalEdge->getTriangle();
                auto [shadow1, shadow2, shadow3] = originalEdge->getShadowTriangle();

                MeshHalfEdge::connectIntoTriangle(shadow1, shadow2, shadow3);

                if (!original1->isBoundary)
                {
                    shadow1->setPairing(original1->pairingEdge->shadowEdge);
                    unvisitedOriginalHalfEdgeList.push(original1->pairingEdge);
                }

                if (!original2->isBoundary)
                {
                    shadow2->setPairing(original2->pairingEdge->shadowEdge);
                    unvisitedOriginalHalfEdgeList.push(original2->pairingEdge);
                }

                if (!original3->isBoundary)
                {
                    shadow3->setPairing(original3->pairingEdge->shadowEdge);
                    unvisitedOriginalHalfEdgeList.push(original3->pairingEdge);
                }

                shadow1->populateVectorWithTriangle(resultCopyList);

            }
        }
    }

    void MeshSplitter::BFSFindRequestedAndIntersecting(SplittablePolygonPtr& intialPolygon, std::vector<SplittablePolygonPtr>& originalSplitMesh, std::vector<SplittablePolygonPtr>& originalNonSplitMesh, SplitState requestedState)
    {
        std::queue<SplittablePolygonPtr> unvisitedPolygonQueue;
        unvisitedPolygonQueue.push(intialPolygon);

        while (!unvisitedPolygonQueue.empty())
        {
            auto polygonPtr = unvisitedPolygonQueue.front();
            unvisitedPolygonQueue.pop();

            //assert(polygonPtr);

            if (!polygonPtr->isVisited)
            {
                polygonPtr->isVisited = true;

                auto polygonSplitState = polygonPtr->GetPolygonSplitState();

                bool polygonAtRequestedState = polygonSplitState == requestedState;
                bool polygonAtIntersection = polygonSplitState == SplitState::Split;

                //place polygon in correct list

                if (polygonAtRequestedState)
                {
                    originalNonSplitMesh.push_back(polygonPtr);
                    //debugHelper.nonIntersectionPolygons.push_back(polygonPtr->localCentroid);
                }
                else if (polygonAtIntersection)
                {
                    originalSplitMesh.push_back(polygonPtr);
                    //debugHelper.intersectionsPolygons.push_back(polygonPtr->localCentroid);
                }

                //only put it on the unvisited list if polygon is at requested state or is intersecting the splitting plane

                if (polygonAtIntersection || polygonAtRequestedState)
                {
                    for (auto edge : polygonPtr->GetMeshEdges())
                    {
                        bool isBoundary = edge->isBoundary;
                        bool hasPairing = (edge->pairingEdge) != nullptr;

                        if (isBoundary && hasPairing)
                        {
                            auto newPolygon = edge->pairingEdge->owner.lock();
                            //assert(newPolygon);
                            unvisitedPolygonQueue.push(newPolygon);
                        }
                    }
                }
            }
        }
    }

    void MeshSplitter::DetectIntersectionIsland(std::vector<SplittablePolygonPtr>& splitPolygons, std::vector<std::vector<SplittablePolygonPtr>>& intersectionIslands)
    {
        for (auto pol : splitPolygons) { pol->isVisited = false; }

        //find first intersection polygon
        SplittablePolygonPtr initialPolygon;
        bool foundUnvisited = FindFirstUnivistedIntersectionPolygon(splitPolygons, initialPolygon);

        //while can find intersection polygon
        while (foundUnvisited)
        {
            std::vector< math::vec3> DEBUG_ONLY_polygonPositions;
            std::vector< SplittablePolygonPtr> intersectionIsland;
            std::queue< SplittablePolygonPtr> unvisitedPolygons;
            unvisitedPolygons.push(initialPolygon);

            while (!unvisitedPolygons.empty())
            {
                auto polygon = unvisitedPolygons.front();
                unvisitedPolygons.pop();

                if (!polygon->isVisited && polygon->GetPolygonSplitState() == SplitState::Split)
                {
                    polygon->isVisited = true;
                    intersectionIsland.push_back(polygon);
                    DEBUG_ONLY_polygonPositions.push_back(polygon->localCentroid);

                    for (auto edge : polygon->GetMeshEdges())
                    {
                        auto pairingEdge = edge->pairingEdge;

                        if (edge->isBoundary && pairingEdge)
                        {
                            if (auto pairingPolygon = pairingEdge->owner.lock())
                            {
                                unvisitedPolygons.push(pairingPolygon);
                            }

                        }
                    }
                }
            }

            intersectionIslands.push_back(intersectionIsland);
            //debugHelper.intersectionIslands.push_back(DEBUG_ONLY_polygonPositions);

            foundUnvisited = FindFirstUnivistedIntersectionPolygon(splitPolygons, initialPolygon);

        }
    }

    void MeshSplitter::SplitPolygon(SplittablePolygonPtr splitPolygon, const math::mat4& transform, const math::vec3 cutPosition,
        const math::vec3 cutNormal, SplitState requestedState, std::vector<IntersectionEdgeInfo>& generatedIntersectionEdges, bool shouldDebug)
    {
        IntersectingPolygonOrganizer polygonOrganizer;
        polygonOrganizer.SplitPolygon(splitPolygon, transform, cutPosition, cutNormal, requestedState, generatedIntersectionEdges,shouldDebug);
    }

}


