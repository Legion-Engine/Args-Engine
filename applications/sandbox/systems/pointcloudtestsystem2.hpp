#pragma once
#include<core/core.hpp>
#include <core/math/math.hpp>
#include <core/logging/logging.hpp>
#include <application/application.hpp>

#include <core/compute/context.hpp>
#include <core/compute/kernel.hpp>


#include <rendering/components/particle_emitter.hpp>

#include <rendering/systems/pointcloudgeneration.hpp>
using namespace legion;
using namespace rendering;
//system to test the point cloud generation system and the point cloud component
class pointcloudtestsystem2 final : public legion::core::System<pointcloudtestsystem2>
{
public:
    ecs::entity_handle player;

    struct player_move : public app::input_axis<player_move> {};
    struct player_strive : public app::input_axis<player_strive> {};
    struct player_fly : public app::input_axis<player_fly> {};
    struct player_look_x : public app::input_axis<player_look_x> {};
    struct player_look_y : public app::input_axis<player_look_y> {};


    virtual void setup() override
    {

        //create particle system material
        material_handle particleMaterial;
        material_handle billboardMat;
        rendering::material_handle rockH;
        //rendering::texture m_normalMap;
        rendering::texture_handle m_normalMap;
        image_handle normal;
        image_handle albedo;
        image_handle albedo2;
        model_handle testHandle;
        material_handle testMat;
        app::window window = m_ecs->world.get_component_handle<app::window>().read();
        {
            app::context_guard guard(window);

            //get mesh
            ModelCache::create_model("cube", "assets://models/Cube.obj"_view);
            ModelCache::create_model("plane", "assets://models/plane.obj"_view);
            ModelCache::create_model("billboard", "assets://models/billboard.obj"_view);

            ModelCache::create_model("uvsphere", "assets://models/uvsphere.obj"_view);
            ModelCache::create_model("sphere", "assets://models/sphere.obj"_view);
            testHandle = ModelCache::create_model("suzanne", "assets://models/suzanne.obj"_view);

            auto colorshader = rendering::ShaderCache::create_shader("color", "assets://shaders/color.shs"_view);
            auto uvShader = rendering::ShaderCache::create_shader("uv", "assets://shaders/uv.shs"_view);
            testMat = rendering::MaterialCache::create_material("uvMat", "assets://shaders/uv.shs"_view);



            auto billBoardsh = rendering::ShaderCache::create_shader("billboard", "assets://shaders/point.shs"_view);
            billboardMat = rendering::MaterialCache::create_material("billboardMat", billBoardsh);
            billboardMat.set_param("fixedSize", false);
            billboardMat.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/sphere.png"_view));
            billboardMat.set_param("_color", math::colors::blue);

            particleMaterial = rendering::MaterialCache::create_material("directional light", colorshader);
            particleMaterial.set_param("color", math::colors::blue);

            normal = ImageCache::create_image("normal image", "assets://textures/nullHeight.png"_view);
            albedo = ImageCache::create_image("albedo image", "assets://textures/rock-albedo.png"_view);
            albedo2 = ImageCache::create_image("albedo2 image", "assets://textures/copper-albedo.png"_view);
        }
        material_handle mat = MaterialCache::get_material("uv");
        mesh_handle uvMesh = MeshCache::get_handle("uvsphere");
        mesh_handle cubeMesh = MeshCache::get_handle("cube");
        mesh_handle sphereMesh = MeshCache::get_handle("sphere");
        mesh_handle suzanneeMesh = MeshCache::get_handle("suzanne");
        mesh_handle sponzaMesh = MeshCache::get_handle("sponza");
        //mesh_handle CarnegieMansion = MeshCache::get_handle("CarnegieMansion");
        auto ent2 = createEntity();
        auto trans2 = ent2.add_components<transform>(position(0, 1, 5), rotation(), scale(0.5f));
        ent2.add_component<point_cloud>(point_cloud(sphereMesh, trans2, billboardMat, albedo, normal, 50000, 0.005f));

        auto ent1 = createEntity();
        auto trans1 = ent1.add_components<transform>(position(1.5f, 1, 5), rotation(), scale(0.5f));
        ent1.add_component<point_cloud>(point_cloud(suzanneeMesh, trans1, billboardMat, albedo2, normal, 50000, 0.005f));


       /* auto ent4 = createEntity();
        ent4.add_components<transform>(position(-1.5f, 1, 0), rotation(), scale(0.5f));
        ent4.add_components<rendering::mesh_renderable>(mesh_filter(uvMesh), rendering::mesh_renderer(testMat));*/

        // ent1.add_component<point_cloud>(point_cloud(suzanneeMesh, trans1, billboardMat, albedo, normal, 10000, 0.025f));

         /*   auto ent3 = createEntity();
            auto trans3 = ent3.add_components<transform>(position(-1.5f, 1, 0), rotation(), scale(0.5f));
            ent3.add_component<point_cloud>(point_cloud(cubeMesh, trans3, billboardMat, albedo, normal, 2000, 0.1f));*/

    }
};





