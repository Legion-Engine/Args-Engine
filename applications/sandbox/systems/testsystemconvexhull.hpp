#pragma once
#include <core/core.hpp>
#include <application/application.hpp>
#include <core/filesystem/filesystem.hpp>
#include <rendering/data/material.hpp>
#include <core/logging/logging.hpp>
#include <rendering/debugrendering.hpp>

#include <rendering/components/renderable.hpp>
#include <physics/components/physics_component.hpp>
#include <physics/components/fracturer.hpp>

#include <rendering/components/particle_emitter.hpp>
#include <physics/halfedgeedge.hpp>
using namespace legion;

struct convex_hull_step : public app::input_action<convex_hull_step> {};
struct convex_hull_draw : public app::input_action<convex_hull_draw> {};
struct convex_hull_info : public app::input_action<convex_hull_info> {};
struct convex_hull_debug : public app::input_action<convex_hull_debug> {};
struct convex_hull_iteration : public app::input_action<convex_hull_iteration> {};

struct followerData
{
    math::vec3 offset;
};

class TestSystemConvexHull final : public System<TestSystemConvexHull>
{
public:
    std::shared_ptr<legion::physics::ConvexCollider> collider = nullptr;

    core::ecs::entity_handle physicsEnt;
    std::vector< core::ecs::entity_handle> followerObjects;

    core::mesh_handle meshH;
    int pStep = 0;

    virtual void setup()
    {
        //app::InputSystem::createBinding<convex_hull_step>(app::inputmap::method::ENTER);
        app::InputSystem::createBinding<convex_hull_draw>(app::inputmap::method::M);
        app::InputSystem::createBinding<convex_hull_info>(app::inputmap::method::I);
        app::InputSystem::createBinding<convex_hull_iteration>(app::inputmap::method::NUM1);
        app::InputSystem::createBinding<convex_hull_debug>(app::inputmap::method::P);

        bindToEvent<convex_hull_step, &TestSystemConvexHull::convexHullStep>();
        bindToEvent<convex_hull_draw, &TestSystemConvexHull::convexHullDraw>();
        bindToEvent<convex_hull_info, &TestSystemConvexHull::convexHullInfo>();
        bindToEvent<convex_hull_iteration, &TestSystemConvexHull::convexHullIteration>();
        bindToEvent< convex_hull_debug, &TestSystemConvexHull::drawConvexHull>();

        createProcess<&TestSystemConvexHull::update>("Physics");

        rendering::model_handle cube;
        rendering::model_handle model;

        rendering::material_handle flatGreen;
        rendering::material_handle vertexColor;
        rendering::material_handle directionalLightMH;
        rendering::material_handle wireFrameH;
        rendering::material_handle solidLegion;

        app::window window = m_ecs->world.get_component_handle<app::window>().read();
        {
        //    app::context_guard guard(window);

        //    cube = rendering::ModelCache::create_model("cube", "assets://models/cube.obj"_view);
        //    model = rendering::ModelCache::create_model("model", "assets://models/sphere.obj"_view);
        //    wireFrameH = rendering::MaterialCache::create_material("wireframe", "assets://shaders/wireframe.shs"_view);
        //    vertexColor = rendering::MaterialCache::create_material("vertexColor", "assets://shaders/vertexcolor.shs"_view);

        //    solidLegion = rendering::MaterialCache::create_material("texture", "assets://shaders/texture.shs"_view);
        //    solidLegion.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/split-test.png"_view));

        //    // Create physics entity
        //    {
        //        physicsEnt = createEntity();
        //        physicsEnt.add_components<rendering::mesh_renderable>(mesh_filter(model.get_mesh()), rendering::mesh_renderer(wireFrameH));
        //        physicsEnt.add_components<transform>(position(0.0f, 4, 0), rotation(), scale(1));
        //        meshH = model.get_mesh();

        //        physicsEnt.add_component<physics::physicsComponent>();
        //        auto rbH = physicsEnt.add_component<physics::rigidbody>();
        //        auto rb = rbH.read();
        //        rb.setMass(1.0f);
        //        rbH.write(rb);
        //    }
        //    // Create physics entity
        //   {
        //        auto ent = createEntity();
        //        ent.add_components<rendering::mesh_renderable>(mesh_filter(cube.get_mesh()), rendering::mesh_renderer(solidLegion));
        //        ent.add_components<transform>(position(0, 2.0f, 0), rotation(), scale(1));
        //        auto pcH = ent.add_component<physics::physicsComponent>();
        //        auto pc = pcH.read();

        //        pc.AddBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f));
        //        pcH.write(pc);
        //    }

        //    /*{
        //        auto ent = createEntity();
        //        ent.add_components<rendering::mesh_renderable>(mesh_filter(cube.get_mesh()), rendering::mesh_renderer(solidLegion));
        //        ent.add_components<transform>(position(7.0f, 2.0f, 0), rotation(), scale(1));
        //        auto pcH = ent.add_component<physics::physicsComponent>();
        //        auto pc = pcH.read();

        //        pc.AddBox(physics::cube_collider_params(2.0f, 2.0f, 2.0f));
        //        pcH.write(pc);
        //    }

        //    {
        //        auto ent = createEntity();
        //        ent.add_components<rendering::mesh_renderable>(mesh_filter(cube.get_mesh()), rendering::mesh_renderer(solidLegion));
        //        ent.add_components<transform>(position(7.0f, 6.0f, -1.35f), rotation(), scale(1));
        //        ent.add_component<physics::rigidbody>();

        //        auto pcH = ent.add_component<physics::physicsComponent>();
        //        auto pc = pcH.read();

        //        auto rot = ent.get_component_handle<rotation>().read();

        //        rot *= math::angleAxis(math::deg2rad(60.0f), math::vec3(1, 0, 0));
        //        rot *= math::angleAxis(math::deg2rad(-45.0f), math::vec3(0, 1, 0));
        //        

        //        ent.write_component(rot);

        //        pc.AddBox(physics::cube_collider_params(1.0f, 1.0f, 1.0f));
        //        pcH.write(pc);
        //    }*/

        //    // Create entity for reference
        //    {
        //        auto ent = createEntity();
        //        ent.add_components<rendering::mesh_renderable>(mesh_filter(cube.get_mesh()), rendering::mesh_renderer(vertexColor));
        //        ent.add_components<transform>(position(5.0f, 0, 0), rotation(), scale(1));
        //    }
        //    // Create entity for reference
        //    {
        //        auto ent = createEntity();
        //        ent.add_components<rendering::mesh_renderable>(mesh_filter(cube.get_mesh()), rendering::mesh_renderer(wireFrameH));
        //        ent.add_components<transform>(position(0, 0, -5.0f), rotation(), scale(1));
        //    }
        //    {
        //        math::vec3 p = math::vec3(11, 4, 5);
        //        std::vector<math::vec3> points{ math::vec3(0,0,0), math::vec3(2, 5, 5), math::vec3(12, 5, 5), math::vec3(10, 0, 0) };
        //        math::mat4 planeMat = math::planeMatrix(points.at(0), points.at(1), points.at(3), math::vec3(0, 0, 0));
        //        std::vector<math::vec3> newPoints{
        //            math::vec3(math::inverse(planeMat) * math::vec4(points.at(0),1)),
        //            math::vec3(math::inverse(planeMat) * math::vec4(points.at(1),1)),
        //            math::vec3(math::inverse(planeMat) * math::vec4(points.at(2),1)),
        //            math::vec3(math::inverse(planeMat) * math::vec4(points.at(3),1))
        //        };
        //        log::debug("mapped: {} {} {} {}", newPoints.at(0), newPoints.at(1), newPoints.at(2), newPoints.at(3));
        //    }
        }
    }

    bool isUpdating = false;
    void update(time::span deltaTime)
    {
        //physics::PhysicsSystem::IsPaused = false;
        //debug::user_projectdrawLine(math::vec3(1, 0, 0), math::vec3(1, 1, 0), math::colors::magenta, 10.0f, 20.0f);
        drawPhysicsColliders();

        auto [posH, rotH, scaleH] = physicsEnt.get_component_handles<transform>();

        if (!isUpdating)
        {
            for (auto ent : followerObjects)
            {
                ent.write_component<position>(posH.read());
                ent.write_component<rotation>(rotH.read());
                ent.write_component<scale>(scaleH.read());
            }
        }
    }
    
    int stepToSee = 0;
    math::vec3 spacing = math::vec3(2.5f, 0, 0);
    int indexToSee = 3;
    void drawConvexHull(convex_hull_debug * action)
    {
        
        //log::debug("drawConvexHull");
        //log::debug("stepToSee {} ", stepToSee);
        if (!action->value)
        {
            ecs::EntityQuery fractureQuery = createQuery<physics::Fracturer>();
            fractureQuery.queryEntities();
            for (auto ent : fractureQuery)
            {
                auto fracturer = ent.read_component<physics::Fracturer>();

                //log::debug("fracturer.transforms {} ", fracturer.transforms.size());

                for (size_t i = 0; i < fracturer.verticesList.size(); i++)
                {
                    if (i != indexToSee) { continue; }
                    auto verticesToUse = fracturer.verticesList.at(i);

                    for (auto& vertex : verticesToUse)
                    {
                        vertex += spacing * stepToSee;
                        math::vec3 drawPos = math::vec3(fracturer.transforms.at(i)[3]) + vertex;
                        debug::user_projectDrawLine(drawPos, drawPos + math::vec3(0, 0.2f, 0), math::colors::red, 12.0f, FLT_MAX);
                    }

                    std::shared_ptr<physics::ConvexCollider> newCollider = std::make_shared<physics::ConvexCollider>();
                    newCollider->debug = false;
                    newCollider->step = stepToSee;
                    //log::debug("ConstructConvexHullWithVertices with step {}", newCollider->step);

                    

                   
                    //vertexDrawCollider(verticesToUse, fracturer.transforms.at(i)[3]);
                    newCollider->ConstructConvexHullWithVertices(verticesToUse, fracturer.transforms.at(i)[3]);
                    newCollider->DrawColliderRepresentation(fracturer.transforms.at(i), math::colors::green, 5.0f, FLT_MAX);
                }
            }
            stepToSee++;
        }
       

    }

    void drawPhysicsColliders()
    {
        static auto physicsQuery = createQuery< physics::physicsComponent>();
        physicsQuery.queryEntities();

        for (auto entity : physicsQuery)
        {
            auto rotationHandle = entity.get_component_handle<rotation>();
            auto positionHandle = entity.get_component_handle<position>();
            auto scaleHandle = entity.get_component_handle<scale>();
            auto physicsComponentHandle = entity.get_component_handle<physics::physicsComponent>();

            bool hasTransform = rotationHandle && positionHandle && scaleHandle;
            bool hasNecessaryComponentsForPhysicsManifold = hasTransform && physicsComponentHandle;

            if (hasNecessaryComponentsForPhysicsManifold)
            {
                auto rbColor = math::color(0.0, 0.5, 0, 1);
                auto statibBlockColor = math::color(0, 1, 0, 1);

                rotation rot = rotationHandle.read();
                position pos = positionHandle.read();
                scale scale = scaleHandle.read();

                auto usedColor = rbColor;
                bool useDepth = false;

                if (entity.get_component_handle<physics::rigidbody>())
                {
                    usedColor = rbColor;
                    //useDepth = true;
                }


                //assemble the local transform matrix of the entity
                math::mat4 localTransform;
                math::compose(localTransform, scale, rot, pos);

                auto physicsComponent = physicsComponentHandle.read();
                if (!physicsComponent.colliders) { continue; }

                for (auto physCollider : *physicsComponent.colliders)
                {
                    //--------------------------------- Draw Collider Outlines ---------------------------------------------//
                    if (!physCollider->shouldBeDrawn) { continue; }
                    //math::vec3 colliderCentroid = pos + math::vec3(localTransform * math::vec4(physCollider->GetLocalCentroid(), 0));
                    //debug::user_projectDrawLine(colliderCentroid, colliderCentroid + math::vec3(0.0f,0.2f,0.0f), math::colors::cyan, 6.0f,0.0f,true);
                    
                    for (auto face : physCollider->GetHalfEdgeFaces())
                    {
                        //face->forEachEdge(drawFunc);
                        physics::HalfEdgeEdge* initialEdge = face->startEdge;
                        physics::HalfEdgeEdge* currentEdge = face->startEdge;
                        math::vec3 worldNormal = (localTransform * math::vec4(face->normal, 0));
                        math::vec3 faceStart = localTransform * math::vec4(face->centroid, 1);
                        math::vec3 faceEnd = faceStart + worldNormal * 0.1f;
                        
                        //debug::user_projectDrawLine(faceStart, faceEnd, math::colors::green, 5.0f);

                        if (!currentEdge) { return; }

                        do
                        {
                            physics::HalfEdgeEdge* edgeToExecuteOn = currentEdge;
                            currentEdge = currentEdge->nextEdge;

                            math::vec3 worldStart = localTransform * math::vec4(edgeToExecuteOn->edgePosition, 1);
                            math::vec3 worldEnd = localTransform * math::vec4(edgeToExecuteOn->nextEdge->edgePosition, 1);

                            debug::user_projectDrawLine(worldStart + worldNormal * 0.005f, worldEnd + worldNormal * 0.005f, usedColor, 3.0f, 0.0f, useDepth);

                        } while (initialEdge != currentEdge && currentEdge != nullptr);
                    }
                }

            }

        }
    }

    void convexHullStep(convex_hull_step* action)
    {
        isUpdating = true;

        if (action->value)
        {
      
            
            auto pc = physicsEnt.read_component<physics::physicsComponent>();

            meshDrawCollider(pc);

            physicsEnt.write_component(pc);

            ++pStep;
        }

        isUpdating = false;
    }

    void meshDrawCollider(physics::physicsComponent& comp)
    {
        for (auto ent : followerObjects)
        {
            m_ecs->destroyEntity(ent);
        }
        followerObjects.clear();

        if (collider == nullptr)
        {
            collider = comp.ConstructConvexHull(meshH);
            PopulateFollowerList();
        }
        else
        {
            collider->shouldBeDrawn = false;
            comp.ConstructConvexHull(meshH, *collider);
            collider->shouldBeDrawn = true;
            PopulateFollowerList();
        }
    }

    void vertexDrawCollider(std::vector<math::vec3> vertices,math::vec3 spacing = math::vec3())
    {
        for (auto ent : followerObjects)
        {
            m_ecs->destroyEntity(ent);
        }
        followerObjects.clear();

        collider->shouldBeDrawn = false;
        collider->ConstructConvexHullWithVertices(vertices, spacing);
        collider->shouldBeDrawn = true;
        PopulateFollowerList();
        collider->step++;

    }

    void PopulateFollowerList()
    {
        app::window window = m_ecs->world.get_component_handle<app::window>().read();
        
        auto [posH, rotH, scaleH] = physicsEnt.get_component_handles<transform>();
        //const math::mat4& trans = math::compose(scaleH.read(), rotH.read(), posH.read());

        for (auto face : collider->GetHalfEdgeFaces())
        {
            
            log::debug("object draw");
            math::vec3 localCentroid = face->centroid;
            std::vector<math::vec3> localVert;

            auto populateVectorLambda = [&localVert](physics::HalfEdgeEdge* edge)
            {
                log::debug("edge {}", math::to_string( edge->edgePosition));
                localVert.push_back(edge->edgePosition);
            };

            face->forEachEdge(populateVectorLambda);

            legion::core::mesh newMesh;
            
            std::vector<math::vec3> vertices;
            std::vector<uint> indices;
            std::vector<math::vec2> uvs;

            /*vertices.push_back(localCentroid);
            vertices.push_back(localCentroid + math::vec3(0,1,0));
            vertices.push_back(localCentroid + math::vec3(0, 0, 1));*/
            for (size_t i = 0; i < localVert.size(); i++)
            {
                vertices.push_back(localVert.at(i));
                vertices.push_back(localVert.at((i+1)%localVert.size()));
                log::debug("using {},{}", i,(i+1));
                vertices.push_back(localCentroid);
            }

            for (size_t i = 0; i < vertices.size(); i++)
            {
                indices.push_back(i);
            }

            for (size_t i = 0; i < vertices.size(); i++)
            {
                uvs.push_back(math::vec2());
            }

            newMesh.vertices = vertices;
            newMesh.indices = indices;
            newMesh.uvs = uvs;

            sub_mesh newSubMesh;
            newSubMesh.indexCount = newMesh.indices.size();
            newSubMesh.indexOffset = 0;

            newMesh.submeshes.push_back(newSubMesh);

            //creaate modelH
            static int count = 0;
            mesh_handle meshH = core::MeshCache::create_mesh("meshh" + std::to_string(count), newMesh);
            auto modelH = rendering::ModelCache::create_model(meshH);
            count++;

            //Create Entity
            auto newEnt = m_ecs->createEntity();
            newEnt.add_components<transform>(posH.read(), rotH.read(), scaleH.read());
            //debug::user_projectdrawLine(posH.read(), posH.read() + math::vec3(0, 1, 0), math::colors::magenta, 20.0f, 20.0f);

            //Create Material
            rendering::material_handle newMat;
            {
                app::context_guard guard(window);
                auto colorShader = rendering::ShaderCache::create_shader("color" + std::to_string(count), "assets://shaders/color.shs"_view);
                newMat = rendering::MaterialCache::create_material("vertex color" + std::to_string(count), colorShader);
                newMat.set_param("color", math::color(math::linearRand(0.25f, 0.7f), math::linearRand(0.25f, 0.7f), math::linearRand(0.25f, 0.7f)));
            }

            mesh_filter meshFilter = mesh_filter(meshH);
            newEnt.add_components<rendering::mesh_renderable>(meshFilter, rendering::mesh_renderer(newMat));


            followerObjects.push_back(newEnt);
        }
    }

    void convexHullDraw(convex_hull_draw* action)
    {
        //log::debug("convexHullDraw(convex_hull_draw* action) ");
        debug::user_projectdebug_line(math::vec3(0, 0, 0), math::vec3(0, 1, 0), math::colors::magenta, 5.0f,20.0f);
        if (action->value)
        {
            if (pStep > 0)
            {
                auto debugDrawEdges = [](legion::physics::HalfEdgeEdge* edge)
                {
                    if (!edge || !edge->nextEdge) return;
                    math::vec3 pos0 = edge->edgePosition;
                    math::vec3 pos1 = edge->nextEdge->edgePosition;
                    debug::drawLine(pos0, pos1, math::colors::red);
                };

                auto faces = collider->GetHalfEdgeFaces();
                for (int i = 0; i < faces.size(); ++i)
                {
                    faces.at(i)->forEachEdge(debugDrawEdges);
                    // Draw normals
                   
                    debug::drawLine(faces.at(i)->centroid, faces.at(i)->centroid + faces.at(i)->normal * 0.3f, math::colors::white);
                }
            }
        }
    }

    void convexHullInfo(convex_hull_info* action)
    {
        if (action->value)
        {
            physics::PhysicsSystem::IsPaused = false;
        }
    }

    int iterationStep = 0;

    void convexHullIteration(convex_hull_iteration* action)
    {
        log::debug("convexHullIteration ");

        if (!action->value)
        {
            auto [posH, rotH, scaleH] = physicsEnt.get_component_handles<transform>();
            const math::mat4 transform = math::compose(scaleH.read(), rotH.read(), posH.read());

            auto faces = collider->GetHalfEdgeFaces();
            for (int i = 0; i < faces.size(); ++i)
            {
                if (i == iterationStep)
                {
                    faces.at(i)->DEBUG_DrawFace(transform, math::colors::red,1.0f);
                }
            }

            iterationStep++;
            
        }
    }

};
