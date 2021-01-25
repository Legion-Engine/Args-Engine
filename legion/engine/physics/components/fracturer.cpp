#include <physics/components/fracturer.hpp>
#include <physics/data/physics_manifold.hpp>
#include <physics/physics_statics.hpp>
#include <physics/colliders/convexcollider.hpp>
#include <physics/data/identifier.hpp>
#include <physics/physics_statics.hpp>

namespace legion::physics
{
    ecs::EcsRegistry* Fracturer::registry = nullptr;

    void Fracturer::HandleFracture(physics_manifold& manifold, bool& manifoldValid,bool isfracturingA)
    {
        if (!IsFractureConditionMet()) { return; }
        //log::debug("manifold invalidated");
        manifoldValid = false;

        //-----------------------------------------------------------------------------------------------------------------------------//
        //------------------------ create a voronoi diagram with a  set of positions for now the diagram-------------------------------//
        //------------------------------- is always as big as the colliders of the physicsComponent------------------------------------//
        //-----------------------------------------------------------------------------------------------------------------------------//

        auto collider = isfracturingA ? manifold.colliderA: manifold.colliderB;
        auto fractureInstigatorEnt = isfracturingA ? manifold.physicsCompA.entity : manifold.physicsCompB.entity;

        auto [min,max] = collider->GetMinMaxWorldAABB();

        math::vec3 difference = max - min;
        math::vec3 differenceQuadrant = difference / 4.0f;

        //-----------------------------------------------------------------------------------------------------------------------------//
                                //Generate a Voronoi Diagram, for now, the points are manually generated //
        //-----------------------------------------------------------------------------------------------------------------------------//

        std::vector<math::vec3> VoronoiPoints;

        math::vec3 first = min + differenceQuadrant ;
        VoronoiPoints.push_back(first);

        math::vec3 second = max - differenceQuadrant;
        VoronoiPoints.push_back(second);

        math::vec3 third = max - (differenceQuadrant * 2);
        VoronoiPoints.push_back(third);

        //math::vec3 fourth = third + math::vec3(0.1f,0,0);
        //VoronoiPoints.push_back(fourth);

        //math::vec3 fifth = third + math::vec3(0.1f, 0, -0.0f);
        //VoronoiPoints.push_back(fifth);

        /*math::vec3 fourth = max - (differenceQuadrant * 2) + math::vec3(0,0.1f,0);
        VoronoiPoints.push_back(fourth);*/

        math::vec3 fifth = min + differenceQuadrant - math::vec3(0, 0.0f, -0.2f);
        VoronoiPoints.push_back(fifth);

        for (auto point : VoronoiPoints)
        {
            debug::user_projectDrawLine(point,
                point + math::vec3(0, 0.1f, 0), math::colors::magenta, 8.0f, FLT_MAX, true);
        }

        auto vectorList = PhysicsStatics::GenerateVoronoi(VoronoiPoints, min.x, max.x, min.y, max.y, min.z, max.z,1,1,1);

        vectorList.pop_back();

        debug::user_projectDrawLine(min,
            min + math::vec3(0, 0.5f, 0), math::colors::red, 8.0f, FLT_MAX, true);

        debug::user_projectDrawLine(max,
            max + math::vec3(0, 0.5f, 0), math::colors::blue, 8.0f, FLT_MAX, true);

        std::vector<std::vector<math::vec3>> groupedPoints(VoronoiPoints.size());

        for (std::vector<math::vec4>& vector : vectorList)
        {
            for (const math::vec4& position : vector)
            {
                int id = position.w;

                //log::debug("position {} id {} ",math::to_string(math::vec3(position)), id);
                groupedPoints.at(id).push_back(position);

            }
        }

        //-----------------------------------------------------------------------------------------------------------------------------//
                                //Using the voronoi points, generate a set of colliders  //
        //-----------------------------------------------------------------------------------------------------------------------------//

        std::vector<std::shared_ptr<ConvexCollider>> voronoiColliders;
        int i = 1;

        for ( std::vector<math::vec3>& vector : groupedPoints)
        {
           // if (i != 1) { continue; }
            math::color debugColor = 
                math::color(math::linearRand(0.0f, 0.3f), math::linearRand(0.0f, 0.3f), math::linearRand(0.0f, 0.3f));
            // i * 2.0f, 0, 0
            math::vec3 debugOffset = math::vec3();
            math::mat4 transform =
                math::compose(math::vec3(1.0f), math::identity<math::quat>(), debugOffset);

            for (math::vec3 vertex : vector)
            {
                //
                math::vec3 vertPos = vertex + debugOffset;
                //debug::user_projectDrawLine(vertPos, vertPos + math::vec3(0,0.5,0), debugColor,10.0f,FLT_MAX);
            }

            auto newCollider = std::make_shared<ConvexCollider>();

            debugVectorcolliders.push_back(newCollider);
            voronoiColliders.push_back(newCollider);

            verticesList.push_back(vector);

            newCollider->ConstructConvexHullWithVertices(vector);


            //transforms.push_back(transform);
            //newCollider->DrawColliderRepresentation(transform,math::colors::green,6.0f,FLT_MAX);
            i++;
        }

        std::vector< FracturerColliderToMeshPairing> colliderToMeshPairings;

        //-----------------------------------------------------------------------------------------------------------------------------//
                                //From the mesh about to be fractured, get the pairs of colliders to Mesh  //
        //-----------------------------------------------------------------------------------------------------------------------------//

        //make sure fractureInstigatorEnt is a direct child of the world, the initial instigator will always
        //be either a direct parent of the world, or a grandchild of the world
        bool isInstigatorDirectParent = fractureInstigatorEnt.get_parent() == registry->world;

        if (!isInstigatorDirectParent)
        {
            //log::debug("instigator is not direct parent");
            fractureInstigatorEnt = fractureInstigatorEnt.get_parent();
        }



        InstantiateColliderMeshPairingWithEntity(fractureInstigatorEnt,
            colliderToMeshPairings);

        for (size_t i = 0; i < fractureInstigatorEnt.child_count() ; i++)
        {
            auto child = fractureInstigatorEnt.get_child(i);

            InstantiateColliderMeshPairingWithEntity(child,
                colliderToMeshPairings);
        }

        //log::debug("colliderToMeshPairings size {} ", colliderToMeshPairings.size());
        //log::debug("voronoiColliderssize {} ", voronoiColliders.size());
        std::vector<ecs::entity_handle> entitiesGenerated;

        int fractureID = 0;
        //for each instantiated convex collider
        for (std::shared_ptr<ConvexCollider> instantiatedVoronoiCollider : voronoiColliders)
        {
            for (auto& meshToColliderPairing : colliderToMeshPairings)
            {
                //check if it collides with one of the colliders in the original physics component
                physics_manifold manifold;
                ConvexConvexCollisionInfo collisionInfo;


                auto ownerEntity = meshToColliderPairing.meshSplitterPairing.
                    entity;
                auto [posH,rotH,scaleH] = ownerEntity.get_component_handles<transform>();
                
     /*           if (!ownerEntity)
                {
                    DebugBreak();
                }*/

                auto transformB = math::compose(scaleH.read(), rotH.read(), posH.read());

                PhysicsStatics::DetectConvexConvexCollision(instantiatedVoronoiCollider.get()
                    , meshToColliderPairing.colliderPair.get(), math::mat4(1.0f), transformB, collisionInfo, manifold);
                
                if (manifold.isColliding)
                {
                    //if (fractureID != 3 || fractureID != 2) { continue; }
                    //{
                        //log::debug("-> Collision Found");

                        std::vector<MeshSplitParams> splittingParams;
                        meshToColliderPairing.GenerateSplittingParamsFromCollider(instantiatedVoronoiCollider, splittingParams);
                        //log::debug("splittingParams {} ", splittingParams.size());

                        for (size_t i = 0; i < splittingParams.size(); i++)
                        {
                            float interpolant = (float)i / splittingParams.size();

                            math::vec3 color = math::color(1, 0, 0) * interpolant;

                            /*debug::user_projectDrawLine(splittingParams.at(i).planePostion
                                , splittingParams.at(i).planePostion + splittingParams.at(i).planeNormal,
                                math::color(color.x, color.y, color.z, 1), 15.0f, FLT_MAX, true);*/
                        }

                        auto splitter = meshToColliderPairing.meshSplitterPairing.read();
                        splitter.MultipleSplitMesh(splittingParams, entitiesGenerated, true, -1);
                        meshToColliderPairing.meshSplitterPairing.write(splitter);

                    //}
                    fractureID++;
                }
                
            
            }
        }
            
        registry->destroyEntity(fractureInstigatorEnt);
        /*registry->destroyEntity(manifold.physicsCompA.entity);
        registry->destroyEntity(manifold.physicsCompB.entity);*/

        //log::debug("entities generated {} ", entitiesGenerated.size());
        int colliderIter = 0;
        for (auto ent : entitiesGenerated)
        {
            
            auto physicsCompHandle = ent.add_component<physicsComponent>();
            auto physicsComp = physicsCompHandle.read();

            auto meshFilter = ent.read_component<mesh_filter>();

            auto vertices = meshFilter.get().second.vertices;

            //log::debug("cch");
            bool debug = colliderIter == entitiesGenerated.size() - 1;
            //log::debug("debug {} ", debug);
            physicsComp.ConstructConvexHull(meshFilter, debug);
            colliderIter++;

            auto posHandle = ent.get_component_handle<position>();

           /* debug::user_projectDrawLine(posHandle.read(),
                posHandle.read() + math::vec3(0,0.2f,0),math::colors::red,15.0f,FLT_MAX,true);*/

            ent.add_component<rigidbody>();
        }

        log::debug("all fragments have convex hulls");
        //for each pair list
            

             //invalidate original collider


        //for each generated collider paired with a newly generated mesh
            //if the collider is in the impact sphere, create a new entity
                
            //else,add it back in the original collider
                
        fractureCount++;
    }

    bool Fracturer::IsFractureConditionMet()
    {
        //TODO the user should be able to create their own fracture condition, but for now
        //, limit it so something can only be fractured once
        log::debug("fractureCount {} ", fractureCount);
        return fractureCount == 0;
    }

    void Fracturer::InitializeVoronoi(ecs::component_handle<physicsComponent> physicsComponent)
    {
        //using all colliders inside the physicsComponent, create a bounding box 

    }

    void Fracturer::InvestigateColliderToMeshPairing(ecs::entity_handle ent, std::vector<FracturerColliderToMeshPairing> colliderToMeshPairings)
    {
        ecs::component_handle<mesh_filter> meshFilterHandle = ent.get_component_handle<mesh_filter>();
        ecs::component_handle<physicsComponent> physicsComponentHandle = ent.get_component_handle<physicsComponent>();

        if (!meshFilterHandle || !physicsComponentHandle) { return; }
    }

    void Fracturer::InstantiateColliderMeshPairingWithEntity(ecs::entity_handle ent,
        std::vector<FracturerColliderToMeshPairing>& colliderToMeshPairings)
    {
        //the fracturer is setup in a way that fragments are all stored in a different entity
        auto physicsCompHandle = ent.get_component_handle<physics::physicsComponent>();
        std::shared_ptr<ConvexCollider> convexCollider = nullptr;

        auto physicsCollider = physicsCompHandle.read().colliders->at(0);

        if (physicsCompHandle)
        {
            convexCollider = std::dynamic_pointer_cast<ConvexCollider>(physicsCollider);
        }

        auto meshSplitterHandle = ent.get_component_handle<MeshSplitter>();
        //assert(meshSplitterHandle);

        if (auto idH = ent.get_component_handle<physics::identifier>())
        {
            auto id = idH.read();
            log::debug("id found {} ", id.id);

        }
        else
        {
            log::debug("NO ID");
        }


        if (meshSplitterHandle && convexCollider)
        {
            FracturerColliderToMeshPairing colliderToMeshPairing(convexCollider, meshSplitterHandle);
            colliderToMeshPairings.push_back(colliderToMeshPairing);
        }


    }

    void FracturerColliderToMeshPairing::GenerateSplittingParamsFromCollider(std::shared_ptr<ConvexCollider> instantiatedCollider, std::vector<physics::MeshSplitParams>& meshSplitParams)
    {
        int until = 5;
        int count = 0;
        for (auto face : instantiatedCollider->GetHalfEdgeFaces())
        {
            //if (count > until) { continue; }
            MeshSplitParams splitParam(face->centroid, face->normal);
            meshSplitParams.push_back(splitParam);
            count++;
        }

    }

}
