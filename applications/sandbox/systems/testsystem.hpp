#pragma once
#include <core/core.hpp>
#include <physics/halfedgeedge.hpp>
#include <application/application.hpp>

#include <physics/components/physics_component.hpp>
#include <physics/components/rigidbody.hpp>
#include <physics/cube_collider_params.hpp>
//#include <physics/data/physics_manifold_precursor.h>
#include <physics/systems/physicssystem.hpp>
#include <physics/halfedgeface.hpp>
//#include <physics/data/penetrationquery.h>


#include <core/compute/context.hpp>
#include <core/compute/kernel.hpp>
#include <core/compute/high_level/function.hpp>
#include <rendering/debugrendering.hpp>

#include <physics/physics_statics.hpp>
#include <physics/data/identifier.hpp>
#include <audio/audio.hpp>
#include <rendering/rendering.hpp>
#include <Voro++/voro++.hh>
#include <Voro++/common.hh>



#include "animation_editor.hpp"
#include "../data/animation.hpp"

using namespace legion;



struct sah
{
    int value;

    sah operator+(const sah& other)
    {
        return { value + other.value };
    }

    sah operator*(const sah& other)
    {
        return { value * other.value };
    }
};

//struct addRB{
//    math::vec3 force = math::vec3(0, 0, 30);
//    float time = 0.0f;
//    float addTime = 5.0f;
//    bool rigidbodyAdded = false;
//};
// Move and strive for the wireframe sphere (which holds the only audio source)
// For testing the movement of audio sources (Spatial audio/doppler)
struct audio_move : public app::input_axis<audio_move> {};
struct audio_strive : public app::input_axis<audio_strive> {};
// Gain and Pitch knob
struct gain_change : public app::input_axis<gain_change> {};
struct pitch_change : public app::input_axis<pitch_change> {};
struct play_audio_source : public app::input_action<play_audio_source> {};
struct pause_audio_source : public app::input_action<pause_audio_source> {};
struct stop_audio_source : public app::input_action<stop_audio_source> {};
struct rewind_audio_source : public app::input_action<rewind_audio_source> {};
struct audio_test_input : public app::input_action<audio_test_input> {};

struct nextEdge_action : public  app::input_action<nextEdge_action> {};
struct nextPairing_action : public  app::input_action<nextPairing_action> {};
//some test stuff just so i can change things

struct physics_test_move : public app::input_axis<physics_test_move> {};

struct light_switch : public app::input_action<light_switch> {};
struct tonemap_switch : public app::input_action<tonemap_switch> {};


struct activate_CRtest2 : public app::input_action<activate_CRtest2> {};
struct activate_CRtest3 : public app::input_action<activate_CRtest3> {};

struct activateFrictionTest : public app::input_action<activateFrictionTest> {};
//
//struct extendedPhysicsContinue : public app::input_action<extendedPhysicsContinue> {};
//struct nextPhysicsTimeStepContinue : public app::input_action<nextPhysicsTimeStepContinue> {};

using namespace legion::core::filesystem::literals;

class TestSystem final : public System<TestSystem>
{
public:
    ecs::entity_handle audioSphereLeft;
    ecs::entity_handle audioSphereRight;
    ecs::entity_handle eventAudio;

    std::vector< ecs::entity_handle > physicsUnitTestCD;
    std::vector< ecs::entity_handle > physicsUnitTestCR;

    //Collision Resolution Tests
    ecs::entity_handle staticToAABBEntLinear;
    ecs::entity_handle staticToAABBEntRotation;
    ecs::entity_handle staticToOBBEnt;
    ecs::entity_handle staticToEdgeEnt;
    ecs::entity_handle staticTo2StackEnt;

    ecs::entity_handle sun;
    rendering::material_handle pbrH;
    rendering::material_handle copperH;
    rendering::material_handle aluminumH;
    rendering::material_handle ironH;
    rendering::material_handle slateH;
    rendering::material_handle rockH;
    rendering::material_handle rock2H;
    rendering::material_handle fabricH;
    rendering::material_handle bogH;
    rendering::material_handle paintH;
    rendering::material_handle skyboxH;
    rendering::material_handle gnomeMH;
    rendering::material_handle textureBillboardH;
    std::vector<gfx::material_handle> suzanneMaterials;

    //Friction Test
    std::vector<ecs::entity_handle> physicsFrictionTestRotators;
    bool rotate = false;

    ecs::entity_handle NoFrictionBody;
    ecs::entity_handle Point3FrictionBody;
    ecs::entity_handle Point6FrictionBody;
    ecs::entity_handle FullFrictionBody;

    //ecs::EntityQuery halfEdgeQuery;

    virtual void setup()
    {

        physics::PrimitiveMesh::SetECSRegistry(m_ecs);

#pragma region Input binding
        app::InputSystem::createBinding<physics_test_move>(app::inputmap::method::LEFT, -1.f);
        app::InputSystem::createBinding<physics_test_move>(app::inputmap::method::RIGHT, 1.f);

        app::InputSystem::createBinding<audio_move>(app::inputmap::method::I, 1.f);
        app::InputSystem::createBinding<audio_move>(app::inputmap::method::K, -1.f);
        app::InputSystem::createBinding<audio_strive>(app::inputmap::method::J, 1.f);
        app::InputSystem::createBinding<audio_strive>(app::inputmap::method::U, 10.f); // Power mode of J
        app::InputSystem::createBinding<audio_strive>(app::inputmap::method::L, -1.f);
        app::InputSystem::createBinding<audio_strive>(app::inputmap::method::O, -10.f); // Power mode of O

        app::InputSystem::createBinding<gain_change>(app::inputmap::method::F10, 0.05f);
        app::InputSystem::createBinding<gain_change>(app::inputmap::method::F9, -0.05f);
        app::InputSystem::createBinding<pitch_change>(app::inputmap::method::APOSTROPHE, 1.0f);
        app::InputSystem::createBinding<pitch_change>(app::inputmap::method::SEMICOLON, -1.0f);

        app::InputSystem::createBinding<play_audio_source>(app::inputmap::method::P);
        app::InputSystem::createBinding<pause_audio_source>(app::inputmap::method::LEFT_BRACKET);
        app::InputSystem::createBinding<stop_audio_source>(app::inputmap::method::RIGHT_BRACKET);
        app::InputSystem::createBinding<rewind_audio_source>(app::inputmap::method::BACKSPACE);
        app::InputSystem::createBinding<audio_test_input>(app::inputmap::method::SLASH);

        app::InputSystem::createBinding< activate_CRtest2>(app::inputmap::method::KP_2);
        app::InputSystem::createBinding< activate_CRtest3>(app::inputmap::method::KP_3);

        app::InputSystem::createBinding< nextEdge_action>(app::inputmap::method::R);
        app::InputSystem::createBinding< nextPairing_action>(app::inputmap::method::T);

        app::InputSystem::createBinding< activateFrictionTest >(app::inputmap::method::KP_4);

        app::InputSystem::createBinding<light_switch>(app::inputmap::method::F);
        app::InputSystem::createBinding<tonemap_switch>(app::inputmap::method::G);

        //app::InputSystem::createBinding< extendedPhysicsContinue>(app::inputmap::method::M);
        //app::InputSystem::createBinding<nextPhysicsTimeStepContinue>(app::inputmap::method::N);

        bindToEvent<physics_test_move, &TestSystem::onUnitPhysicsUnitTestMove>();

        bindToEvent<light_switch, &TestSystem::onLightSwitch>();
        bindToEvent<tonemap_switch, &TestSystem::onTonemapSwitch>();

        bindToEvent<audio_move, &TestSystem::onSphereAAMove>();
        bindToEvent<audio_strive, &TestSystem::onSphereAAStrive>();
        bindToEvent<gain_change, &TestSystem::onGainChange>();
        bindToEvent<pitch_change, &TestSystem::onPitchChange>();

        bindToEvent<play_audio_source, &TestSystem::playAudioSource>();
        bindToEvent<pause_audio_source, &TestSystem::pauseAudioSource>();
        bindToEvent<stop_audio_source, &TestSystem::stopAudioSource>();
        bindToEvent<rewind_audio_source, &TestSystem::rewindAudioSource>();

        bindToEvent<physics::trigger_event, &TestSystem::testPhysicsEvent>();

        bindToEvent<audio_test_input, &TestSystem::audioTestInput>();

        //bindToEvent<physics_split_test, &TestSystem::OnSplit>();

        //collision resolution test
        bindToEvent< activate_CRtest2, &TestSystem::onActivateUnitTest2>();
        bindToEvent< activate_CRtest3, &TestSystem::onActivateUnitTest3>();

        //friction test
        //bindToEvent< activateFrictionTest, &TestSystem::FrictionTestActivate>();

        bindToEvent< nextEdge_action, &TestSystem::OnNextEdge>();
        bindToEvent<nextPairing_action, &TestSystem::OnNextPair>();

        //bindToEvent< extendedPhysicsContinue, &TestSystem::onExtendedPhysicsContinueRequest>();
        //bindToEvent<nextPhysicsTimeStepContinue, &TestSystem::onNextPhysicsTimeStepRequest>();

        bindToEvent<ext::void_animation_event, &TestSystem::onVoidAnimationEvent>();

#pragma endregion

#pragma region Model and material loading
        const float additionalLightIntensity = 0.5f;

        rendering::model_handle directionalLightH;
        rendering::model_handle spotLightH;
        rendering::model_handle pointLightH;
        rendering::model_handle audioSourceH;
        rendering::model_handle cubeH;
        rendering::model_handle sphereH;
        rendering::model_handle suzanneH;
        rendering::model_handle gnomeH;
        rendering::model_handle uvsphereH;
        rendering::model_handle axesH;
        rendering::model_handle submeshtestH;
        rendering::model_handle planeH;
        rendering::model_handle magneticLowH;
        rendering::model_handle cylinderH;
        rendering::model_handle billboardH;
        //rendering::model_handle cylinderH;

        rendering::material_handle wireframeH;
        rendering::material_handle vertexColorH;

        rendering::material_handle uvH;
        rendering::material_handle texture2H;
        rendering::material_handle directionalLightMH;
        rendering::material_handle spotLightMH;
        rendering::material_handle pointLightMH;
        rendering::material_handle gizmoMH;
        rendering::material_handle normalH;
        rendering::material_handle billboardMH;
        rendering::material_handle particleMH;
        rendering::material_handle fixedSizeBillboardMH;
        rendering::material_handle fixedSizeParticleMH;

        app::window window = m_ecs->world.get_component_handle<app::window>().read();

        {
            std::lock_guard guard(*window.lock);
            app::ContextHelper::makeContextCurrent(window);

            directionalLightH = rendering::ModelCache::create_model("directional light", "assets://models/directional-light.obj"_view);
            spotLightH = rendering::ModelCache::create_model("spot light", "assets://models/spot-light.obj"_view);
            pointLightH = rendering::ModelCache::create_model("point light", "assets://models/point-light.obj"_view);
            audioSourceH = rendering::ModelCache::create_model("audio source", "assets://models/audio-source.obj"_view);
            cubeH = rendering::ModelCache::create_model("cube", "assets://models/cube.obj"_view);
            cylinderH = rendering::ModelCache::create_model("cylinder", "assets://models/cylinder.obj"_view);
            sphereH = rendering::ModelCache::create_model("sphere", "assets://models/sphere.obj"_view);
            suzanneH = rendering::ModelCache::create_model("suzanne", "assets://models/suzanne-test.obj"_view, suzanneMaterials);
            gnomeH = rendering::ModelCache::create_model("gnome", "assets://models/wizardgnomeretop.obj"_view);
            uvsphereH = rendering::ModelCache::create_model("uvsphere", "assets://models/uvsphere.obj"_view);
            axesH = rendering::ModelCache::create_model("axes", "assets://models/xyz.obj"_view);
            submeshtestH = rendering::ModelCache::create_model("submeshtest", "assets://models/submeshtest.obj"_view);
            planeH = rendering::ModelCache::create_model("plane", "assets://models/plane.obj"_view);
            magneticLowH = rendering::ModelCache::create_model("complexMesh", "assets://models/magneticLevelLow.obj"_view);
            billboardH = rendering::ModelCache::create_model("billboard", "assets://models/billboard.obj"_view);
            //cylinderH = rendering::ModelCache::create_model("cylinder","assets://models/cylinder.obj"_view);

            wireframeH = rendering::MaterialCache::create_material("wireframe", "assets://shaders/wireframe.shs"_view);
            vertexColorH = rendering::MaterialCache::create_material("vertex color", "assets://shaders/vertexcolor.shs"_view);
            uvH = rendering::MaterialCache::create_material("uv", "assets://shaders/uv.shs"_view);

            auto lightshader = rendering::ShaderCache::create_shader("light", "assets://shaders/light.shs"_view);
            directionalLightMH = rendering::MaterialCache::create_material("directional light", lightshader);
            directionalLightMH.set_param("color", math::color(1, 1, 0.8f));
            directionalLightMH.set_param("intensity", 1.f);

            spotLightMH = rendering::MaterialCache::create_material("spot light", lightshader);
            spotLightMH.set_param("color", math::colors::green);
            spotLightMH.set_param("intensity", additionalLightIntensity);

            pointLightMH = rendering::MaterialCache::create_material("point light", lightshader);
            pointLightMH.set_param("color", math::colors::red);
            pointLightMH.set_param("intensity", additionalLightIntensity);

            auto colorshader = rendering::ShaderCache::create_shader("color", "assets://shaders/color.shs"_view);
            gizmoMH = rendering::MaterialCache::create_material("gizmo", colorshader);
            gizmoMH.set_param("color", math::colors::lightgrey);

            textureBillboardH = rendering::MaterialCache::create_material("texture billboard", "assets://shaders/point.shs"_view);

            billboardMH = rendering::MaterialCache::create_material("billboard", "assets://shaders/billboard.shs"_view);
            billboardMH.set_param("_texture", rendering::TextureCache::create_texture("engine://resources/default/albedo"_view));
            billboardMH.set_param("fixedSize", false);

            particleMH = rendering::MaterialCache::create_material("particle", "assets://shaders/particle.shs"_view);
            particleMH.set_param("fixedSize", false);

            fixedSizeBillboardMH = rendering::MaterialCache::create_material("fixed size billboard", "assets://shaders/billboard.shs"_view);
            fixedSizeBillboardMH.set_param("_texture", rendering::TextureCache::create_texture("engine://resources/default/albedo"_view));
            fixedSizeBillboardMH.set_param("fixedSize", true);

            fixedSizeParticleMH = rendering::MaterialCache::create_material("fixed size particle", "assets://shaders/particle.shs"_view);
            fixedSizeParticleMH.set_param("fixedSize", true);

            texture2H = rendering::MaterialCache::create_material("texture", "assets://shaders/texture.shs"_view);
            texture2H.set_param("_texture", rendering::TextureCache::create_texture("assets://textures/split-test.png"_view));

            auto pbrShader = rendering::ShaderCache::create_shader("pbr", "assets://shaders/pbr.shs"_view);
            pbrH = rendering::MaterialCache::create_material("pbr", pbrShader);
            pbrH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("engine://resources/default/albedo"_view));
            pbrH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("engine://resources/default/normalHeight"_view));
            pbrH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("engine://resources/default/MRDAo"_view));
            pbrH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("engine://resources/default/emissive"_view));
            pbrH.set_param(SV_HEIGHTSCALE, 1.f);
            pbrH.set_param("discardExcess", false);
            pbrH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            copperH = rendering::MaterialCache::create_material("copper", pbrShader);
            copperH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/copper/copper-albedo-512.png"_view));
            copperH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/copper/copper-normalHeight-512.png"_view));
            copperH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/copper/copper-MRDAo-512.png"_view));
            copperH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/copper/copper-emissive-512.png"_view));
            copperH.set_param(SV_HEIGHTSCALE, 1.f);
            copperH.set_param("discardExcess", false);
            copperH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            aluminumH = rendering::MaterialCache::create_material("aluminum", pbrShader);
            aluminumH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/aluminum/aluminum-albedo-512.png"_view));
            aluminumH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/aluminum/aluminum-normalHeight-512.png"_view));
            aluminumH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/aluminum/aluminum-MRDAo-512.png"_view));
            aluminumH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/aluminum/aluminum-emissive-512.png"_view));
            aluminumH.set_param(SV_HEIGHTSCALE, 1.f);
            aluminumH.set_param("discardExcess", false);
            aluminumH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            ironH = rendering::MaterialCache::create_material("iron", pbrShader);
            ironH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/iron/rustediron-albedo-512.png"_view));
            ironH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/iron/rustediron-normalHeight-512.png"_view));
            ironH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/iron/rustediron-MRDAo-512.png"_view));
            ironH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/iron/rustediron-emissive-512.png"_view));
            ironH.set_param(SV_HEIGHTSCALE, 1.f);
            ironH.set_param("discardExcess", false);
            ironH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            slateH = rendering::MaterialCache::create_material("slate", pbrShader);
            slateH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/slate/slate-albedo-512.png"_view));
            slateH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/slate/slate-normalHeight-512.png"_view));
            slateH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/slate/slate-MRDAo-512.png"_view));
            slateH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/slate/slate-emissive-512.png"_view));
            slateH.set_param(SV_HEIGHTSCALE, 1.f);
            slateH.set_param("discardExcess", true);
            slateH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            rockH = rendering::MaterialCache::create_material("rock", pbrShader);
            rockH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/rock/rock-albedo-512.png"_view));
            rockH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/rock/rock-normalHeight-512.png"_view));
            rockH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/rock/rock-MRDAo-512.png"_view));
            rockH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/rock/rock-emissive-512.png"_view));
            rockH.set_param(SV_HEIGHTSCALE, 1.f);
            rockH.set_param("discardExcess", true);
            rockH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            rock2H = rendering::MaterialCache::create_material("rock 2", pbrShader);
            rock2H.set_param(SV_ALBEDO, rendering::TextureCache::get_handle("rock-albedo-512.png"));
            rock2H.set_param(SV_NORMALHEIGHT, rendering::TextureCache::get_handle("rock-normalHeight-512.png"));
            rock2H.set_param(SV_MRDAO, rendering::TextureCache::get_handle("rock-MRDAo-512.png"));
            rock2H.set_param(SV_EMISSIVE, rendering::TextureCache::get_handle("rock-emissive-512.png"));
            rock2H.set_param(SV_HEIGHTSCALE, 1.f);
            rock2H.set_param("discardExcess", false);
            rock2H.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            fabricH = rendering::MaterialCache::create_material("fabric", pbrShader);
            fabricH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/fabric/fabric-lowres-albedo-512.png"_view));
            fabricH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/fabric/fabric-lowres-normalHeight-512.png"_view));
            fabricH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/fabric/fabric-lowres-MRDAo-512.png"_view));
            fabricH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/fabric/fabric-lowres-emissive-512.png"_view));
            fabricH.set_param(SV_HEIGHTSCALE, 1.f);
            fabricH.set_param("discardExcess", false);
            fabricH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            bogH = rendering::MaterialCache::create_material("bog", pbrShader);
            bogH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/bog/bog-albedo-512.png"_view));
            bogH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/bog/bog-normalHeight-512.png"_view));
            bogH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/bog/bog-MRDAo-512.png"_view));
            bogH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/bog/bog-emissive-512.png"_view));
            bogH.set_param(SV_HEIGHTSCALE, 1.f);
            bogH.set_param("discardExcess", true);
            bogH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            paintH = rendering::MaterialCache::create_material("paint", pbrShader);
            paintH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/paint/paint-peeling-albedo-512.png"_view));
            paintH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/paint/paint-peeling-normalHeight-512.png"_view));
            paintH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/paint/paint-peeling-MRDAo-512.png"_view));
            paintH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/paint/paint-peeling-emissive-512.png"_view));
            paintH.set_param(SV_HEIGHTSCALE, 1.f);
            paintH.set_param("discardExcess", false);
            paintH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            gnomeMH = rendering::MaterialCache::create_material("gnome", pbrShader);
            gnomeMH.set_param(SV_ALBEDO, rendering::TextureCache::create_texture("assets://textures/warlock/warlock-albedo-512.png"_view));
            gnomeMH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("assets://textures/warlock/warlock-normalHeight-512.png"_view));
            gnomeMH.set_param(SV_MRDAO, rendering::TextureCache::create_texture("assets://textures/warlock/warlock-MRDAo-512.png"_view));
            gnomeMH.set_param(SV_EMISSIVE, rendering::TextureCache::create_texture("assets://textures/warlock/warlock-emissive-512.png"_view));
            gnomeMH.set_param(SV_HEIGHTSCALE, 0.f);
            gnomeMH.set_param("discardExcess", false);
            gnomeMH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            normalH = rendering::MaterialCache::create_material("normal", "assets://shaders/normal.shs"_view);
            normalH.set_param(SV_NORMALHEIGHT, rendering::TextureCache::create_texture("engine://resources/default/normalHeight"_view));

            skyboxH = rendering::MaterialCache::create_material("skybox", "assets://shaders/skybox.shs"_view);
            skyboxH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

            app::ContextHelper::makeContextCurrent(nullptr);
        }
#pragma endregion

#pragma region Entities
        {
            auto ent = createEntity();
            ent.add_component(rendering::mesh_renderer(slateH, planeH));
            ent.add_components<transform>(position(0, 0.01f, 0), rotation(), scale(10));

        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(copperH));
            ent.add_components<transform>(position(10, 0.01f, 0), rotation(), scale(10));
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(aluminumH));
            ent.add_components<transform>(position(10, 0.01f, 10), rotation(), scale(10));
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(ironH));
            ent.add_components<transform>(position(10, 0.01f, -10), rotation(), scale(10));
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(fabricH));
            ent.add_components<transform>(position(-10, 0.01f, 0), rotation(), scale(10));
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(bogH));
            ent.add_components<transform>(position(-10, 0.01f, 10), rotation(), scale(10));
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(pbrH));
            ent.add_components<transform>(position(0, 0.01f, 10), rotation(), scale(10));
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(rockH));
            ent.add_components<transform>(position(0, 0.01f, -10), rotation(), scale(10));
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(paintH));
            ent.add_components<transform>(position(-10, 0.01f, -10), rotation(), scale(10));
        }

        {
            sun = createEntity();
            sun.add_components<rendering::mesh_renderable>(mesh_filter(directionalLightH.get_mesh()), rendering::mesh_renderer(directionalLightMH));
            sun.add_component<rendering::light>(rendering::light::directional(math::color(1, 1, 0.8f), 10.f));
            sun.add_components<transform>(position(10, 10, 10), rotation::lookat(math::vec3(1, 1, 1), math::vec3::zero), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(billboardMH, billboardH));
            ent.add_components<transform>(position(-10, 0, 10), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(fixedSizeBillboardMH, billboardH));
            ent.add_components<transform>(position(-10, 0, 8), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(particleMH, billboardH));
            ent.add_components<transform>(position(-8, 0, 10), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(fixedSizeParticleMH, billboardH));
            ent.add_components<transform>(position(-8, 0, 8), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(billboardMH, billboardH));
            ent.add_components<transform>(position(-11, 0.5, 10), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(fixedSizeBillboardMH, billboardH));
            ent.add_components<transform>(position(-11, 0.5, 8), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(particleMH, billboardH));
            ent.add_components<transform>(position(-9, 0.5, 10), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_component(gfx::mesh_renderer(fixedSizeParticleMH, billboardH));
            ent.add_components<transform>(position(-9, 0.5, 8), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(spotLightH.get_mesh()), rendering::mesh_renderer(spotLightMH));
            ent.add_component<rendering::light>(rendering::light::spot(math::colors::green, math::deg2rad(45.f), additionalLightIntensity, 50.f));
            ent.add_components<transform>(position(-10, 0.5, -10), rotation::lookat(math::vec3(0, 0, -1), math::vec3::zero), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(spotLightH.get_mesh()), rendering::mesh_renderer(spotLightMH));
            ent.add_component<rendering::light>(rendering::light::spot(math::colors::green, math::deg2rad(45.f), additionalLightIntensity, 50.f));
            ent.add_components<transform>(position(0, 0.5, -10), rotation::lookat(math::vec3(0, 0, -1), math::vec3::zero), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(spotLightH.get_mesh()), rendering::mesh_renderer(spotLightMH));
            ent.add_component<rendering::light>(rendering::light::spot(math::colors::green, math::deg2rad(45.f), additionalLightIntensity, 50.f));
            ent.add_components<transform>(position(10, 0.5, -10), rotation::lookat(math::vec3(0, 0, -1), math::vec3::zero), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(pointLightH.get_mesh()), rendering::mesh_renderer(pointLightMH));
            ent.add_component<rendering::light>(rendering::light::point(math::colors::red, additionalLightIntensity, 50.f));
            ent.add_components<transform>(position(0, 1, 0), rotation(), scale());
        }


        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(pointLightH.get_mesh()), rendering::mesh_renderer(pointLightMH));
            ent.add_component<rendering::light>(rendering::light::point(math::colors::red, additionalLightIntensity, 50.f));
            ent.add_components<transform>(position(-10, 1, 0), rotation(), scale());
        }


        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(pointLightH.get_mesh()), rendering::mesh_renderer(pointLightMH));
            ent.add_component<rendering::light>(rendering::light::point(math::colors::red, additionalLightIntensity, 50.f));
            ent.add_components<transform>(position(10, 1, 0), rotation(), scale());
        }


        {
            auto ent = createEntity();
            if (suzanneMaterials.size() > 0)
            {
                suzanneMaterials[0].set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                ent.add_components<rendering::mesh_renderable>(mesh_filter(suzanneH.get_mesh()), rendering::mesh_renderer(suzanneMaterials[0]));
            }
            else
                ent.add_components<rendering::mesh_renderable>(mesh_filter(suzanneH.get_mesh()), rendering::mesh_renderer(gfx::invalid_material_handle));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, 5.1f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(suzanneH.get_mesh()), rendering::mesh_renderer(wireframeH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, 8.1f), rotation(), scale());
        }

        {
            auto ent = m_ecs->createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(suzanneH.get_mesh()), rendering::mesh_renderer(copperH));
            ent.add_component<sah>({});

            ent.add_components<transform>(position(0, 3, 11.1f), rotation(), scale());
        }

        {
            auto ent = m_ecs->createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(gnomeH.get_mesh()), rendering::mesh_renderer(gnomeMH));
            ent.add_component<sah>({});

            ent.add_components<transform>(position(), rotation(), scale());
        }

        /*   {
               auto ent = m_ecs->createEntity();
               ent.add_component<sah>();
               m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent), { gnomeH, normalH });

               auto [positionH, rotationH, scaleH] = m_ecs->createComponent<transform>(ent);
               positionH.write(math::vec3(0, 2, 15.1f));
               scaleH.write(math::vec3(1.f));
           }*/

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(submeshtestH.get_mesh()), rendering::mesh_renderer(pbrH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 10, 0), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(axesH.get_mesh()), rendering::mesh_renderer(vertexColorH));
            ent.add_components<transform>();
        }
        //no audio for you 
#if !defined(SUPER_LOW_POWER)
        {
            eventAudio = createEntity();

            auto segment = audio::AudioSegmentCache::createAudioSegment("e",
                fs::view("assets://audio/fx/explosion.wav"));
            audio::audio_source source;
            source.setAudioHandle(segment);
            source.disableSpatialAudio();

            eventAudio.add_components<transform>();
            eventAudio.add_component<audio::audio_source>(source);
        }
#endif
        //position positions[1000];
        //for (int i = 0; i < 1000; i++)
        //{
        //    positions[i] = position(math::linearRand(math::vec3(-10, -21, -10), math::vec3(10, -1, 10)));
        //}

        //time::timer clock;
        //time::timer entityClock;
        //time::time_span<time64> entityTime;
        //for (int i = 0; i < 00; i++)
        //{
        //    auto ent = createEntity();
        //    ent.add_components<rendering::mesh_renderable>(mesh_filter(sphereH.get_mesh()), rendering::mesh_renderer(pbrH));
        //    ent.add_component<sah>({});
        //    entityClock.start();
        //    ent.add_components<transform>(position(math::linearRand(math::vec3(-10, -21, -10), math::vec3(10, -1, 10))), rotation(), scale());
        //    entityTime += entityClock.end();
        //}
        //auto elapsed = clock.elapsedTime();
        //log::debug("Making entities took {}ms", elapsed.milliseconds());
        //log::debug("Creating transforms took {}ms", entityTime.milliseconds());

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(pbrH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(5.1f, 9, 0), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(sphereH.get_mesh()), rendering::mesh_renderer(copperH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, -5.1f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(sphereH.get_mesh()), rendering::mesh_renderer(aluminumH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, -8.f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(sphereH.get_mesh()), rendering::mesh_renderer(ironH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, -2.2f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(sphereH.get_mesh()), rendering::mesh_renderer(rock2H));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(4, 3, -8.f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(sphereH.get_mesh()), rendering::mesh_renderer(fabricH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(4, 3, -5.1f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(sphereH.get_mesh()), rendering::mesh_renderer(paintH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(4, 3, -2.2f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(uvsphereH.get_mesh()), rendering::mesh_renderer(copperH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, -3.6f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(uvsphereH.get_mesh()), rendering::mesh_renderer(aluminumH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, -6.5f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(uvsphereH.get_mesh()), rendering::mesh_renderer(ironH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(0, 3, -0.7f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(uvsphereH.get_mesh()), rendering::mesh_renderer(rock2H));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(4, 3, -6.5f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(uvsphereH.get_mesh()), rendering::mesh_renderer(fabricH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(4, 3, -3.6f), rotation(), scale());
        }

        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(uvsphereH.get_mesh()), rendering::mesh_renderer(paintH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(4, 3, -0.7f), rotation(), scale());
            auto ent2 = ent.clone();
            auto pos = ent2.get_component_handle<position>().read();
            pos.y = 6;
            ent2.get_component_handle<position>().write(pos);
        }
#if defined(LEGION_DEBUG)
        for (int i = 0; i < 2000; i++)
#else
        for (int i = 0; i < 20000; i++)
#endif
        {
            auto ent = createEntity();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(billboardH.get_mesh()), rendering::mesh_renderer(textureBillboardH));
            ent.add_component<sah>({});
            ent.add_components<transform>(position(math::linearRand(math::vec3(40, -21, -10), math::vec3(60, -1, 10))), rotation(), scale(0.1f));
        }

        //audioSphereLeft setup
        //{
        //    audioSphereLeft = createEntity();
        //    audioSphereLeft.add_components<rendering::mesh_renderable>(mesh_filter(audioSourceH.get_mesh()), rendering::mesh_renderer(gizmoMH));
        //    audioSphereLeft.add_components<transform>(position(-5, 1, 10), rotation(), scale(0.5));

        //    auto segment = audio::AudioSegmentCache::createAudioSegment("kilogram", "assets://audio/kilogram-of-scotland_stereo32.wav"_view, { audio::audio_import_settings::channel_processing_setting::split_channels });

        //    audio::audio_source source;
        //    source.setAudioHandle(segment);
        //    audioSphereLeft.add_component<audio::audio_source>(source);
        //}
        ////audioSphereRight setup
        //{
        //    audioSphereRight = createEntity();
        //    audioSphereRight.add_components<rendering::mesh_renderable>(mesh_filter(audioSourceH.get_mesh()), rendering::mesh_renderer(gizmoMH));
        //    audioSphereRight.add_components<transform>(position(5, 1, 10), rotation(), scale(0.5));

        //    auto segment = audio::AudioSegmentCache::getAudioSegment("kilogram_channel1");

        //    audio::audio_source source;
        //    source.setAudioHandle(segment);
        //    source.setLooping(true);
        //    audioSphereRight.add_component<audio::audio_source>(source);
       // }
#pragma endregion

        //---------------------------------------------------------- Physics Collision Unit Test -------------------------------------------------------------------//

        //setupPhysicsCDUnitTest(cubeH, wireframeH);

        //----------- Rigidbody-Collider AABB Test------------//

        //physics::HalfEdgeFinder finder;
        //std::vector<std::shared_ptr<physics::MeshHalfEdge>> physics;
        //std::vector<int> indices;
        //std::vector<math::vec3> vertices;
        //finder.FindHalfEdge(indices, vertices, math::mat4(1.0), physics);
        //setupPhysicsFrictionUnitTest(cubeH, uvH);

        //setupPhysicsStackingUnitTest(cubeH,uvH,TextureH);

        //setupMeshSplitterTest(planeH,cubeH, cylinderH, magneticLowH,texture2H);
//        setupPhysicsCompositeTest(cubeH, texture2H);
        //setupPhysicsCRUnitTest(cubeH, texture2H);

        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;
        //setupPhysicsCRUnitTest(cubeH, uvH);


        //auto sceneEntity = createEntity();
        //std::vector<ecs::entity_handle> children;
        //for (size_type i = 0; i < m_ecs->world.child_count(); i++)
        //{
        //    children.push_back(m_ecs->world.get_child(i));
        //}
        //for (auto child : children)
        //{
        //    if (child != sceneEntity)
        //    {
        //        child.set_parent(sceneEntity);
        //    }
        //}

        //scenemanagement::SceneManager::createScene("Main", sceneEntity);

        //sceneEntity.destroy();

        //scenemanagement::SceneManager::loadScene("ImposterFlake");

        //CreateCubeStack(3, 2, 2, math::vec3(0, -3.0f, 8.0f), math::vec3(1, 1, 1)
        //    ,cubeParams, 0.1f, cubeH, wireframeH);
        //physicsUpdate(time::span deltaTime)
        createProcess<&TestSystem::update>("Update");
        //createProcess<&TestSystem::drawInterval>("Update");
        //      createProcess<&TestSystem::physicsUpdate>("Physics", 0.02f);
    }

    void setupMeshSplitterTest(rendering::model_handle planeH, rendering::model_handle cubeH
        , rendering::model_handle cylinderH, rendering::model_handle complexH, rendering::material_handle TextureH)
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 2.0f;
        cubeParams.width = 2.0f;
        cubeParams.height = 2.0f;
        ecs::entity_handle cubeSplit2;
        {
            auto splitter = m_ecs->createEntity();
            cubeSplit2 = splitter;

            auto entPhyHande = splitter.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);

            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            splitter.add_components<rendering::mesh_renderable>(mesh_filter(planeH.get_mesh()), rendering::mesh_renderer(TextureH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(splitter);
            positionH.write(math::vec3(36, 1.0f, 10.0f));
            scaleH.write(math::vec3(0.01f));

            auto rotation = rotationH.read();

            rotation *= math::angleAxis(math::deg2rad(-60.0f), math::vec3(1, 0, 0));

            splitter.write_component(rotation);

        }
        //Cube split plane
        ecs::entity_handle cubeSplit;
        {
            /* auto splitter = m_ecs->createEntity();
             cubeSplit = splitter;

             auto entPhyHande = splitter.add_component<physics::physicsComponent>();

             physics::physicsComponent physicsComponent2;
             physics::physicsComponent::init(physicsComponent2);

             physicsComponent2.AddBox(cubeParams);

             entPhyHande.write(physicsComponent2);

             splitter.add_components<rendering::mesh_renderable>(planeH.get_mesh(), rendering::mesh_renderer(TextureH));

             auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(splitter);
             positionH.write(math::vec3(37, 1.5f, 10.0f));
             scaleH.write(math::vec3(0.01f));

             auto rotation = rotationH.read();

             rotation *= math::angleAxis(math::deg2rad(60.0f), math::vec3(1, 0, 0));

             splitter.write_component(rotation);*/

        }




        //Cube 
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();
            //ent.add_component<addRB>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(TextureH));
            //renderableHandle.write({ cubeH,TextureH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(37, 1.5f, 10.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "AABBRbStable";
            idHandle.write(id);

            auto finderH = ent.add_component<physics::MeshSplitter>();

            auto finder = finderH.read();
            //finder.splitTester.push_back(cubeSplit);
            finder.splitTester.push_back(cubeSplit2);
            finder.InitializePolygons(ent);
            finderH.write(finder);

        }

        //Split plane
        ecs::entity_handle cylinderSplit;
        {
            //auto splitterCylinder = m_ecs->createEntity();
            //cylinderSplit = splitterCylinder;

            ////auto crb = m_ecs->createComponent<physics::rigidbody>(staticToAABBEnt);
            ////auto rbHandle = staticToAABBEnt.add_component<physics::rigidbody>();
            //splitterCylinder.add_components<rendering::mesh_renderable>(planeH.get_mesh(), rendering::mesh_renderer(TextureH));

            //auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(splitterCylinder);
            //positionH.write(math::vec3(37, 1.5f, 15.0f));
            //scaleH.write(math::vec3(0.02f));

        }

        //Cylinder
        {
            //auto ent = m_ecs->createEntity();

            //auto entPhyHande = ent.add_component<physics::physicsComponent>();

            //physics::physicsComponent physicsComponent;
            //physics::physicsComponent::init(physicsComponent);

            //physicsComponent.AddBox(cubeParams);
            //entPhyHande.write(physicsComponent);

            //

            ////auto crb = m_ecs->createComponent<physics::rigidbody>(staticToAABBEnt);
            ////auto rbHandle = staticToAABBEnt.add_component<physics::rigidbody>();

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(ent);
            //renderableHandle.write({ cylinderH,TextureH });

            //auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            //positionH.write(math::vec3(37, 1.5f, 15.0f));
            //scaleH.write(math::vec3(1.0f));

            //auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            //auto id = idHandle.read();
            //id.id = "AABBRbStable";
            //idHandle.write(id);


            //auto finderH = ent.add_component<physics::MeshSplitter>();

            //auto finder = finderH.read();
            //finder.splitTester = cylinderSplit;
            //finder.InitializePolygons(ent);
            //finderH.write(finder);
        }

        //Complex Mesh
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);

            entPhyHande.write(physicsComponent2);

            //auto crb = m_ecs->createComponent<physics::rigidbody>(staticToAABBEnt);
            //auto rbHandle = staticToAABBEnt.add_component<physics::rigidbody>();
            ent.add_components<rendering::mesh_renderable>(mesh_filter(complexH.get_mesh()), rendering::mesh_renderer(TextureH));


            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(37, 1.5f, 20.0f));
            scaleH.write(math::vec3(5.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "AABBRbStable";
            idHandle.write(id);


            /*auto finderH = ent.add_component<physics::HalfEdgeFinder>();

            auto finder = finderH.read();
            finder.InitializePolygons(ent);
            finderH.write(finder);*/
        }

        //Complex Mesh

        createProcess<&TestSystem::update>("Update");
        ext::AnimationEditor::onRenderCustomEventGUI(ext::void_animation_event::id, [this](id_type id, ext::animation_event_base* ebase)
            {
                imgui::base::Text("Void Animations Custom Edit Frontend!");

                static bool showBaseRenderLayer = false;
                if (imgui::base::Button(fmt::format("Show Base Renderer [{}]", showBaseRenderLayer).c_str()))
                {
                    showBaseRenderLayer = !showBaseRenderLayer;
                }
                return showBaseRenderLayer;
            });
        //createProcess<&TestSystem::drawInterval>("TestChain");
    }



    void onVoidAnimationEvent(ext::void_animation_event* evnt)
    {
        auto source = eventAudio.read_component<audio::audio_source>();
        source.play();
        eventAudio.get_component_handle<audio::audio_source>().write(source);

        log::debug("received void animation_event");
    }

    void testPhysicsEvent(physics::trigger_event* evnt)
    {
        log::debug("received trigger event {}", evnt->manifold.isColliding);
    }

    void setupPhysicsCDUnitTest(rendering::model_handle cubeH, rendering::material_handle wireframeH)
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 2.0f;
        cubeParams.width = 2.0f;
        cubeParams.height = 2.0f;

        physics::cube_collider_params cubeParams2;
        cubeParams2.breadth = 3.0f;
        cubeParams2.width = 3.0f;
        cubeParams2.height = 3.0f;

        //----------- AABB to AABB Test(Single Axis)  ------------//
        //**
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(0, -3.0f, 8.0f));
            scaleH.write(math::vec3(1.0f));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsUnitTestCD.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = true;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(3.0, -3.0f, 8.0f));
            scaleH.write(math::vec3(1.0f));
        }



        //----------- AABB to AABB Test  ------------//
        //**
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(0, -3.0f, 5.0f));
            scaleH.write(math::vec3(1.0f));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsUnitTestCD.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = true;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(3.0, -2.0f, 4.0f));
            scaleH.write(math::vec3(1.0f));
        }
        //*/

        //----------- AABB to OBB Test  ------------//
        //**
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(0, -3.0f, -2.0f));
            scaleH.write(math::vec3(1.0f));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsUnitTestCD.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = true;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(3.0, -3.0f, -2.0f));

            auto rot = rotationH.read();
            rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rot *= math::angleAxis(45.f, math::vec3(0, 0, 1));

            rotationH.write(rot);

            scaleH.write(math::vec3(1.0f));
        }
        //*/

        //----------- OBB to OBB Test  ------------//
        //**
        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(0, -3.0f, -7.0f));

            auto rot = rotationH.read();
            rot *= math::angleAxis(20.f, math::vec3(0, 1, 0));
            rotationH.write(rot);

            scaleH.write(math::vec3(1.0f));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsUnitTestCD.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = true;
            entPhyHande.write(physicsComponent2);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(3.0, -3.0f, -7.0f));

            auto rot = rotationH.read();
            rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rot *= math::angleAxis(45.f, math::vec3(0, 0, 1));

            rotationH.write(rot);

            scaleH.write(math::vec3(1.0f));
        }
        //*/

        //----------- OBB Edge-Edge Test  ------------//
        //**
        {
            auto ent = m_ecs->createEntity();
            //physicsUnitTestObjects.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(0, -3.0f, -12.0f));

            auto rot = rotationH.read();
            rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rotationH.write(rot);

            scaleH.write(math::vec3(1.0f));
        }
        {
            auto ent = m_ecs->createEntity();
            physicsUnitTestCD.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = true;
            entPhyHande.write(physicsComponent2);


            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(3.0, -3.0f, -13.0f));

            auto rot = rotationH.read();
            rot *= math::angleAxis(45.f, math::vec3(1, 0, 0));
            rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rotationH.write(rot);

            scaleH.write(math::vec3(1.0f));
        }
        //*/
    }
    //10,0,15

    void setupPhysicsCRUnitTest(rendering::model_handle cubeH, rendering::material_handle wireframeH)
    {
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;

        physics::cube_collider_params staticBlockParams;
        staticBlockParams.breadth = 2.5f;
        staticBlockParams.width = 2.5f;
        staticBlockParams.height = 1.0f;

        float testPos = 10.0f;
        //----------- Static Block To AABB Body Stability Test ------------//
        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "AABBStaticStable";
            idHandle.write(idComp);

        }

        {
            staticToAABBEntLinear = m_ecs->createEntity();
            physicsUnitTestCD.push_back(staticToAABBEntLinear);
            auto entPhyHande = staticToAABBEntLinear.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            entPhyHande.write(physicsComponent2);

            //auto crb = m_ecs->createComponent<physics::rigidbody>(staticToAABBEnt);
            //auto rbHandle = staticToAABBEnt.add_component<physics::rigidbody>();

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToAABBEnt);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToAABBEntLinear);
            positionH.write(math::vec3(testPos, -0.0f, 15.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToAABBEntLinear);
            auto id = idHandle.read();
            id.id = "AABBRbStable";
            idHandle.write(id);
        }

        //{
        //    auto ent = m_ecs->createEntity();
        //    //physicsUnitTestCD.push_back(staticToAABBEntLinear);
        //    auto entPhyHande = ent.add_component<physics::physicsComponent>();

        //    physics::physicsComponent physicsComponent2;
        //    physics::physicsComponent::init(physicsComponent2);


        //    physicsComponent2.AddBox(cubeParams);
        //    entPhyHande.write(physicsComponent2);

        //    auto crb = m_ecs->createComponent<physics::rigidbody>(ent);
        //    auto rbHandle = ent.add_component<physics::rigidbody>();

        //    //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToAABBEnt);
        //    //renderableHandle.write({ cubeH), wireframeH });

        //    auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
        //    positionH.write(math::vec3(testPos+0.5f, -1.0f, 15.0f));
        //    scaleH.write(math::vec3(1.0f));

        //    auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
        //    auto id = idHandle.read();
        //    id.id = "AABBRbStable";
        //    idHandle.write(id);
        //}

        //----------- Static Block To AABB Body Rotation Test------------//

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 8.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 8.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "AABBStatic";
            idHandle.write(idComp);


        }

        {
            staticToAABBEntRotation = m_ecs->createEntity();
            physicsUnitTestCD.push_back(staticToAABBEntRotation);
            auto entPhyHande = staticToAABBEntRotation.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);

            physicsComponent2.AddBox(cubeParams);
            entPhyHande.write(physicsComponent2);

            //auto crb = m_ecs->createComponent<physics::rigidbody>(staticToAABBEnt);
            //auto rbHandle = staticToAABBEnt.add_component<physics::rigidbody>();

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToAABBEnt);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToAABBEntRotation);
            positionH.write(math::vec3(testPos - 2.7f, -0.0f, 8.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToAABBEntRotation);
            auto id = idHandle.read();
            id.id = "AABBRb";
            idHandle.write(id);
        }

        //----------- Static Block To Rotated Body ------------//

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 2.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);



            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 2.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "OBBStatic";
            idHandle.write(id);
        }

        {
            staticToOBBEnt = m_ecs->createEntity();

            auto entPhyHande = staticToOBBEnt.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);


            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(staticToOBBEnt);
            //renderableHandle.write({ cubeH, wireframeH });
            staticToOBBEnt.add_components<rendering::mesh_renderable>
                (mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToOBBEnt);
            positionH.write(math::vec3(testPos, -0.0f, 2.0f));
            scaleH.write(math::vec3(1.0f));

            auto rot = rotationH.read();
            /*rot *= math::angleAxis(math::radians(90.f), math::vec3(0, 0, 1));
            rot *= math::angleAxis(math::radians(40.f), math::vec3(1, 0, 0));
            rot *= math::angleAxis(math::radians(42.f), math::vec3(0, 1, 0));*/
            //rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rotationH.write(rot);

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToOBBEnt);
            auto id = idHandle.read();
            id.id = "NON_STATIC";
            idHandle.write(id);


            //log::debug("rb.angularVelocity {}", rb.angularVelocity);

        }

        //----------- Static Block To Rotated Body Edge ------------//

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, -4.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);



            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, -4.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "OBBFoundationStaticEdge";
            idHandle.write(id);
        }

        {
            staticToEdgeEnt = m_ecs->createEntity();

            auto entPhyHande = staticToEdgeEnt.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            entPhyHande.write(physicsComponent2);


            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(staticToEdgeEnt);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(staticToEdgeEnt);
            positionH.write(math::vec3(testPos + 3.0f, 2.0f, -4.0f));
            scaleH.write(math::vec3(1.0f));

            auto rot = rotationH.read();
            rot *= math::angleAxis(45.f, math::vec3(1, 0, 0));
            rot *= math::angleAxis(45.f, math::vec3(0, 1, 0));
            rotationH.write(rot);

            auto idHandle = m_ecs->createComponent<physics::identifier>(staticToEdgeEnt);
            auto id = idHandle.read();
            id.id = "OBBRb";
            idHandle.write(id);


            //log::debug("rb.angularVelocity {}", rb.angularVelocity);

        }


    }
    //15,0,15
    void setupPhysicsFrictionUnitTest(rendering::model_handle cubeH, rendering::material_handle wireframeH)
    {
        float testPos = 20.f;
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;

        physics::cube_collider_params staticBlockParams;
        staticBlockParams.breadth = 5.0f;
        staticBlockParams.width = 5.0f;
        staticBlockParams.height = 2.0f;

        //NO Friction
        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "StaticNoFriction";
            idHandle.write(idComp);


        }

        {
            NoFrictionBody = m_ecs->createEntity();

            auto entPhyHande = NoFrictionBody.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);

            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(NoFrictionBody);
            positionH.write(math::vec3(testPos - 1.0f, -1.5f, 15.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(NoFrictionBody);
            auto idComp = idHandle.read();
            idComp.id = "NoFriction";
            idHandle.write(idComp);

        }

        //0.3f Friction

        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 8.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 8.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "StaticNoFriction";
            idHandle.write(idComp);


        }

        {
            Point3FrictionBody = m_ecs->createEntity();

            auto entPhyHande = Point3FrictionBody.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(Point3FrictionBody);
            positionH.write(math::vec3(testPos - 1.0f, -1.5f, 8.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(Point3FrictionBody);
            auto idComp = idHandle.read();
            idComp.id = "Point3Friction";
            idHandle.write(idComp);

        }


        //0.6f Friction

        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 1.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 1.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "StaticNoFriction";
            idHandle.write(idComp);


        }

        {
            Point6FrictionBody = m_ecs->createEntity();

            auto entPhyHande = Point6FrictionBody.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(Point6FrictionBody);
            positionH.write(math::vec3(testPos - 1.0f, -1.5f, 1.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(Point6FrictionBody);
            auto idComp = idHandle.read();
            idComp.id = "NoFriction";
            idHandle.write(idComp);

        }

        //1.0f Friction

        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, -6.0f));
            scaleH.write(math::vec3(2.5f, 1.0f, 2.5f));

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));
        }

        {
            auto ent = m_ecs->createEntity();
            physicsFrictionTestRotators.push_back(ent);
            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, -6.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto idComp = idHandle.read();
            idComp.id = "StaticNoFriction";
            idHandle.write(idComp);


        }

        {
            FullFrictionBody = m_ecs->createEntity();

            auto entPhyHande = FullFrictionBody.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            //auto renderableHandle = m_ecs->createComponent<rendering::mesh_renderable>(mesh_filter(ent);
            //renderableHandle.write({ cubeH), wireframeH });

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(FullFrictionBody);
            positionH.write(math::vec3(testPos - 1.0f, -1.5f, -6.0f));
            scaleH.write(math::vec3(1.0f));

            auto idHandle = m_ecs->createComponent<physics::identifier>(FullFrictionBody);
            auto idComp = idHandle.read();
            idComp.id = "FullFrictionBody";
            idHandle.write(idComp);

        }

        //


    }
    //20,0,15

    void setupPhysicsCompositeTest(rendering::model_handle cubeH, rendering::material_handle TextureH)
    {
        float testPos = 20.f;
        physics::cube_collider_params cubeParams;
        cubeParams.breadth = 1.0f;
        cubeParams.width = 1.0f;
        cubeParams.height = 1.0f;

        physics::cube_collider_params staticBlockParams;
        staticBlockParams.breadth = 2.0f;
        staticBlockParams.width = 2.0f;
        staticBlockParams.height = 1.0f;

        //BLOCK
        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(2.0f, 1.0f, 2.0f));

            //auto entPhyHande = ent.add_component<physics::physicsComponent>();

           /* physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);*/

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "STATIC_BLOCK";
            idHandle.write(id);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(TextureH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -3.0f, 15.0f));
            scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);


            physicsComponent2.AddBox(staticBlockParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "STATIC_BLOCK";
            idHandle.write(id);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(TextureH));
        }

        {
            auto ent = m_ecs->createEntity();

            auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
            positionH.write(math::vec3(testPos, -1.0f, 15.0f));
            scaleH.write(math::vec3(1.0f, 1.0f, 1.0f));

            auto entPhyHande = ent.add_component<physics::physicsComponent>();

            physics::physicsComponent physicsComponent2;
            physics::physicsComponent::init(physicsComponent2);

            physicsComponent2.AddBox(cubeParams);
            physicsComponent2.isTrigger = false;
            entPhyHande.write(physicsComponent2);

            ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(TextureH));

            auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
            auto id = idHandle.read();
            id.id = "NON_STATIC";
            idHandle.write(id);

            //rotation *= math::angleAxis(math::deg2rad(30.0f), math::vec3(1, 0, 0));

            auto rbHandle = ent.add_component<physics::rigidbody>();
        }





    }


#pragma region input stuff
    void onLightSwitch(light_switch* action)
    {
        size_type dispatch_size = 2000;
        std::vector<size_type> numbers;
        numbers.resize(dispatch_size);

        const float scalar = 2.f;

        time::timer t;
        t.start();
        m_scheduler->queueJobs(dispatch_size, [&]()
            {
                id_type id = async::this_job::get_id();
                // because of the resize we are only doing read operations on the vector which are thread safe.
                numbers[id] += id * scalar;
            }).then(dispatch_size, [&]()
                {
                    id_type id = async::this_job::get_id();
                    numbers[id] += id * scalar;
                }).wait();

                auto elapsed = t.end();
                for (int i = 0; i < 5; i++)
                {
                    log::debug(numbers[i]);
                }
                log::debug("...");
                log::debug("dispatches took {}ms", elapsed.milliseconds());

                static bool on = true;

                static auto decalH = gfx::MaterialCache::get_material("decal");

                if (!action->value)
                {
                    //auto light = sun.read_component<rendering::light>();
                    if (on)
                    {
                        /*light.set_intensity(0.f);
                        sun.write_component(light);*/

                        if (sun)
                            sun.destroy();

                        decalH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        pbrH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        copperH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        aluminumH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        ironH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        slateH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        rockH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        rock2H.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        fabricH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        bogH.set_param("skycolor", math::color(0.002f, 0.003f, 0.0035f));
                        paintH.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        skyboxH.set_param("skycolor", math::color(0.002f, 0.003f, 0.0035f));

                        for (auto& mat : suzanneMaterials)
                        {
                            mat.set_param("skycolor", math::color(0.005f, 0.0055f, 0.0065f));
                        }
                    }
                    else
                    {
                        if (!sun)
                        {
                            sun = createEntity();
                            sun.add_components<rendering::mesh_renderable>(
                                mesh_filter(MeshCache::get_handle("directional light")),
                                rendering::mesh_renderer(rendering::MaterialCache::get_material("directional light")));

                            sun.add_component<rendering::light>(rendering::light::directional(math::color(1, 1, 0.8f), 10.f));
                            sun.add_components<transform>(position(10, 10, 10), rotation::lookat(math::vec3(1, 1, 1), math::vec3::zero), scale());
                        }

                        decalH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        pbrH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        copperH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        aluminumH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        ironH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        slateH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        rockH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        rock2H.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        fabricH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        bogH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        paintH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        skyboxH.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));

                        for (auto& mat : suzanneMaterials)
                        {
                            mat.set_param("skycolor", math::color(0.1f, 0.3f, 1.0f));
                        }
                    }
                    on = !on;
                }
    }

    void onTonemapSwitch(tonemap_switch* action)
    {
        static gfx::tonemapping_type algorithm = gfx::tonemapping_type::aces;

        if (!action->value)
        {
            switch (algorithm)
            {
            case gfx::tonemapping_type::aces:
                gfx::Tonemapping::setAlgorithm(gfx::tonemapping_type::reinhard);
                algorithm = gfx::tonemapping_type::reinhard;
                log::debug("Reinhard tonemapping");
                break;
            case gfx::tonemapping_type::reinhard:
                gfx::Tonemapping::setAlgorithm(gfx::tonemapping_type::reinhard_jodie);
                algorithm = gfx::tonemapping_type::reinhard_jodie;
                log::debug("Reinhard Jodie tonemapping");
                break;
            case gfx::tonemapping_type::reinhard_jodie:
                gfx::Tonemapping::setAlgorithm(gfx::tonemapping_type::legion);
                algorithm = gfx::tonemapping_type::legion;
                log::debug("Legion tonemapping");
                break;
            case gfx::tonemapping_type::legion:
                gfx::Tonemapping::setAlgorithm(gfx::tonemapping_type::unreal3);
                algorithm = gfx::tonemapping_type::unreal3;
                log::debug("Unreal3 tonemapping");
                break;
            case gfx::tonemapping_type::unreal3:
                gfx::Tonemapping::setAlgorithm(gfx::tonemapping_type::aces);
                algorithm = gfx::tonemapping_type::aces;
                log::debug("ACES tonemapping");
                break;
            default:
                gfx::Tonemapping::setAlgorithm(gfx::tonemapping_type::legion);
                algorithm = gfx::tonemapping_type::legion;
                log::debug("Legion tonemapping");
                break;
            }
        }
    }

    void onSphereAAMove(audio_move* action)
    {
        auto posH = audioSphereLeft.get_component_handle<position>();
        math::vec3 move = math::vec3(0.f, 0.f, 1.f);
        move = math::normalize(move * math::vec3(1, 0, 1)) * action->value * action->input_delta * 10.f;
        posH.fetch_add(move);
    }

    void onSphereAAStrive(audio_strive* action)
    {
        auto posH = audioSphereLeft.get_component_handle<position>();
        math::vec3 move = math::vec3(1.f, 0.f, 0.f);
        move = math::normalize(move * math::vec3(1, 0, 1)) * action->value * action->input_delta * 10.f;
        posH.fetch_add(move);
    }

    void onGainChange(gain_change* action)
    {
        if (action->value == 0) return;
        using namespace audio;
        auto sourceH = audioSphereLeft.get_component_handle<audio_source>();
        audio_source source = sourceH.read();
        float g = source.getGain() + action->value * action->input_delta * 10.0f;
        source.setGain(g);
        sourceH.write(source);

        sourceH = audioSphereRight.get_component_handle<audio_source>();
        source = sourceH.read();
        g = source.getGain() + action->value * action->input_delta * 10.0f;
        source.setGain(g);
        sourceH.write(source);
    }

    void onPitchChange(pitch_change* action)
    {
        if (action->value == 0) return;
        using namespace audio;
        auto sourceH = audioSphereLeft.get_component_handle<audio_source>();
        audio_source source = sourceH.read();
        const float p = source.getPitch() + action->value * action->input_delta * 10.0f;
        source.setPitch(p);
        sourceH.write(source);
    }

    void audioTestInput(audio_test_input* action)
    {
        using namespace audio;
        auto sourceH = audioSphereLeft.get_component_handle<audio_source>();
        audio_source source = sourceH.read();
        source.disableSpatialAudio();
        sourceH.write(source);
    }

    void onUnitPhysicsUnitTestMove(physics_test_move* action)
    {
        for (auto entity : physicsUnitTestCD)
        {
            auto posHandle = entity.get_component_handle<position>();

            auto pos = posHandle.read();
            pos.x += 0.005 * action->value;


            posHandle.write(pos);

        }
    }

    void playAudioSource(play_audio_source* action)
    {
        {
            auto sourceH = audioSphereLeft.get_component_handle<audio::audio_source>();
            audio::audio_source source = sourceH.read();
            source.play();
            sourceH.write(source);
        }
        {
            auto sourceH = audioSphereRight.get_component_handle<audio::audio_source>();
            audio::audio_source source = sourceH.read();
            source.play();
            sourceH.write(source);
        }
    }

    void pauseAudioSource(pause_audio_source* action)
    {
        {
            auto sourceH = audioSphereLeft.get_component_handle<audio::audio_source>();
            audio::audio_source source = sourceH.read();
            source.pause();
            sourceH.write(source);
        }
        {
            auto sourceH = audioSphereRight.get_component_handle<audio::audio_source>();
            audio::audio_source source = sourceH.read();
            source.pause();
            sourceH.write(source);
        }
    }

    void stopAudioSource(stop_audio_source* action)
    {
        {
            auto sourceH = audioSphereLeft.get_component_handle<audio::audio_source>();
            audio::audio_source source = sourceH.read();
            source.stop();
            sourceH.write(source);
        }
        {
            auto sourceH = audioSphereRight.get_component_handle<audio::audio_source>();
            audio::audio_source source = sourceH.read();
            source.stop();
            sourceH.write(source);
        }
    }

    void rewindAudioSource(rewind_audio_source* action)
    {
        auto sourceH = audioSphereLeft.get_component_handle<audio::audio_source>();
        audio::audio_source source = sourceH.read();
        source.rewind();
        sourceH.write(source);
    }
#pragma endregion



    void update(time::span deltaTime)
    {
        static float timer = 0;

        static float avgdt = deltaTime;
        avgdt = (avgdt + deltaTime) / 2.f;
        timer += deltaTime;
        if (timer > 1.f)
        {
            timer -= 1.f;
            log::debug("frametime {}ms, fps {}", avgdt, 1.f / avgdt);
        }

        //static auto sahQuery = createQuery<sah, rotation, position, scale>();
        static auto sahQuery = createQuery<sah, position>();
        sahQuery.queryEntities();

        //auto& rotations = sahQuery.get<rotation>();
        auto& positions = sahQuery.get<position>();
        //auto& scales = sahQuery.get<scale>();

        float dt = deltaTime;

        m_scheduler->queueJobs(sahQuery.size(), [&]()
            {
                id_type idx = async::this_job::get_id();
                //auto& rot = rotations[idx];
                auto& pos = positions[idx];
                //auto& scale = scales[idx];
                float t = time::mainClock.elapsedTime();
                pos += math::vec3(math::sin(t) * 0.01f, math::sin(t + 1.f) * 0.01f, math::sin(t - 1.f) * 0.01f);
                //rot = math::angleAxis(math::deg2rad(45.f * dt), rot.up()) * rot;
            }).wait();
            sahQuery.submit<position>();
            //sahQuery.submit<rotation>();

            //if (rotate && !physics::PhysicsSystem::IsPaused)
            //{
            //    for (auto entity : physicsFrictionTestRotators)
            //    {
            //        auto rot = entity.read_component<rotation>();

            //        rot *= math::angleAxis(math::deg2rad(-20.f * deltaTime), math::vec3(0, 0, 1));

            //        entity.write_component(rot);
            //    }
            //}

            //static auto posQuery = createQuery<position>();

            //posQuery.queryEntities();
            //for (auto entity : posQuery)
            //{
            //    auto pos = entity.read_component<position>();

            //    debug::drawLine(pos, pos + math::vec3(0,1,0), math::colors::blue,10.0f,0.0f);

            //}



            //if (buffer > 1.f)
            //{
            //    buffer -= 1.f;

            //    for (auto entity : query)
            //    {
            //        auto comp = entity.get_component_handle<sah>();
            //        std::cout << "component value: " << comp.read().value << std::endl;
            //    }

            //    std::cout << "Hi! " << (frameCount / accumulated) << "fps " << deltaTime.milliseconds() << "ms" << std::endl;
            //}
    }

    //void physicsUpdate(time::span deltaTime)
    //{
    //    static ecs::EntityQuery halfEdgeQuery = createQuery<physics::MeshSplitter>();

    //    halfEdgeQuery.queryEntities();
    //    //log::debug("halfEdgeQuery.size() {} ", halfEdgeQuery.size());
    //    for (auto entity : halfEdgeQuery)
    //    {
    //        auto edgeFinderH = entity.get_component_handle<physics::MeshSplitter>();
    //        auto [posH, rotH, scaleH] = entity.get_component_handles<transform>();

    //        math::mat4 transform = math::compose(scaleH.read(), rotH.read(), posH.read());

    //        auto splitter = edgeFinderH.read();

    //        //auto edgePtr = splitter.edgeFinder.currentPtr;

    //        //math::vec3 worldPos = transform * math::vec4(edgePtr->position, 1);
    //        //math::vec3 worldNextPos = transform * math::vec4(edgePtr->nextEdge->position, 1);

    //        //debug::drawLine(worldPos, worldNextPos, math::colors::red, 1.0f, 0.0f, true);

    //        //debug::drawLine(worldPos, worldPos + math::vec3(0, 0.1f, 0), math::colors::green, 5.0f, 0.0f, true);
    //        //debug::drawLine(worldNextPos, worldNextPos + math::vec3(0, 0.1f, 0), math::colors::blue, 5.0f, 0.0f, true);

    //        auto getEdge = entity.get_component_handle<physics::identifier>();

    //        for (size_t i = 0; i < splitter.debugHelper.intersectionIslands.size(); i++)
    //        {
    //            auto maxColor = splitter.debugHelper.colors.size();
    //            math::color color = splitter.debugHelper.colors[i % maxColor];

    //            auto island = splitter.debugHelper.intersectionIslands.at(i);

    //            for (auto pos : island)
    //            {
    //                math::vec3 worldIntersect = transform * math::vec4(pos, 1);
    //                debug::drawLine(worldIntersect, worldIntersect + math::vec3(0, 0.1f, 0), color, 10.0f, 0.0f);
    //            }


    //        }

    //        /* for (auto intersectingPosition : edgeFinder.debugHelper.intersectionsPolygons)
    //         {
    //             math::vec3 worldIntersect = transform * math::vec4(intersectingPosition, 1);
    //             debug::drawLine(worldIntersect, worldIntersect + math::vec3(0, 0.1f, 0), math::colors::blue, 10.0f, 0.0f);
    //         }*/

    //        for (auto intersectingPosition : splitter.debugHelper.nonIntersectionPolygons)
    //        {
    //            math::vec3 worldIntersect = transform * math::vec4(intersectingPosition, 1);
    //            debug::drawLine(worldIntersect, worldIntersect + math::vec3(0, 0.1f, 0), math::colors::yellow, 10.0f, 0.0f);
    //        }


    //        //log::debug("Count boundary polygon {} ");
    //        for (auto polygon : splitter.meshPolygons)
    //        {
    //            int boundaryCount = 0;
    //            math::vec3 worldCentroid = transform * math::vec4(polygon->localCentroid, 1);

    //            for (auto edge : polygon->GetMeshEdges())
    //            {
    //                if (edge->isBoundary)
    //                {
    //                    boundaryCount++;

    //                    math::vec3 worldEdgePos = transform * math::vec4(edge->position, 1);
    //                    math::vec3 worldEdgeNextPos = transform * math::vec4(edge->nextEdge->position, 1);

    //                    math::vec3 edgeToCentroid = (worldCentroid - worldEdgePos) * 0.05f;
    //                    math::vec3 nextEdgeToCentroid = (worldCentroid - worldEdgeNextPos) * 0.05f;

    //                    debug::drawLine(worldEdgePos + edgeToCentroid
    //                        , worldEdgeNextPos + nextEdgeToCentroid, polygon->debugColor, 5.0f, 0.0f, false);
    //                }

    //            }
    //            /*              math::vec3 normalWorld = transform * math::vec4(polygon->localNormal, 0);
    //                          debug::drawLine(worldCentroid
    //                              , worldCentroid + (normalWorld), polygon->debugColor, 5.0f, 0.0f, false);*/

    //                              // log::debug("polygon boundaryCount {} ", boundaryCount);

    //        }

    //        auto& boundaryInfoList = splitter.debugHelper.boundaryEdgesForPolygon;

    //        /* debug::drawLine(splitter.debugHelper.cuttingSetting.first
    //             , splitter.debugHelper.cuttingSetting.first + (splitter.debugHelper.cuttingSetting.second) * 2.0f, math::colors::cyan, 5.0f, 0.0f, false);*/



    //        for (size_t i = 0; i < boundaryInfoList.size(); i++)
    //        {
    //            auto& boundaryInfo = boundaryInfoList[i];
    //            math::color color = boundaryInfo.drawColor;

    //            if (i != splitter.debugHelper.polygonToDisplay) { continue; }

    //            math::vec3 polygonNormalOffset = boundaryInfo.worldNormal * 0.01f;

    //            debug::drawLine(boundaryInfo.intersectionPoints.first
    //                , boundaryInfo.intersectionPoints.second, math::colors::magenta, 10.0f, 0.0f, false);

    //            for (int j = 0; j < boundaryInfo.boundaryEdges.size(); j++)
    //            {
    //                auto edge = boundaryInfo.boundaryEdges.at(j);

    //                math::vec3 worldEdgePos = transform * math::vec4(edge->position, 1);
    //                math::vec3 worldEdgeNextPos = transform * math::vec4(edge->nextEdge->position, 1);

    //                float interpolant = (float)j / boundaryInfo.boundaryEdges.size();

    //                debug::drawLine(worldEdgePos
    //                    , worldEdgeNextPos, math::lerp(color, math::colors::black, interpolant), 10.0f, 0.0f, false);

    //            }

    //            math::vec3 basePos = boundaryInfo.base + polygonNormalOffset;
    //            debug::drawLine(basePos
    //                , boundaryInfo.base + math::vec3(0, 0.1f, 0) + polygonNormalOffset, math::colors::red, 10.0f, 0.0f, false);

    //            debug::drawLine(boundaryInfo.prevSupport + polygonNormalOffset
    //                , boundaryInfo.prevSupport + math::vec3(0, 0.1f, 0) + polygonNormalOffset, math::colors::green, 10.0f, 0.0f, false);

    //            debug::drawLine(boundaryInfo.nextSupport + polygonNormalOffset
    //                , boundaryInfo.nextSupport + math::vec3(0, 0.1f, 0) + polygonNormalOffset, math::colors::blue, 10.0f, 0.0f, false);

    //            debug::drawLine(boundaryInfo.intersectionEdge + polygonNormalOffset
    //                , boundaryInfo.intersectionEdge + math::vec3(0, 0.1f, 0) + polygonNormalOffset, math::colors::magenta, 10.0f, 0.0f, false);
    //        }

    //    }
    //}

    void differentInterval(time::span deltaTime)
    {
        static time::span buffer;
        static int frameCount;
        static time::span accumulated;

        buffer += deltaTime;
        accumulated += deltaTime;
        frameCount++;

        math::vec2 v;
        v.x = 10;
        v.y = 20;

        if (buffer > 1.f)
        {
            buffer -= 1.f;
            //std::cout << "This is a fixed interval!! " << (frameCount / accumulated) << "fps " << deltaTime.milliseconds() << "ms" << std::endl;
        }
    }

    void differentThread(time::span deltaTime)
    {
        static auto query = createQuery<sah>();

        static time::span buffer;
        static int frameCount;
        static time::span accumulated;

        buffer += deltaTime;
        accumulated += deltaTime;
        frameCount++;

        /* if (buffer > 1.f)
         {
             buffer -= 1.f;

             for (auto entity : query)
             {
                 auto comp = entity.get_component_handle<sah>();
                 std::cout << "component value on different thread: " << comp.read().value << std::endl;
             }

             std::cout << "This is a different thread!! " << (frameCount / accumulated) << "fps " << deltaTime.milliseconds() << "ms" << std::endl;
         }*/

         //if (accumulated > 10.f)
         //{
         //	std::cout << "raising exit event" << std::endl;
         //	raiseEvent<events::exit>();
         //	//throw legion_exception_msg("hehehe fuck you >:D");
         //}
    }

    //void drawInterval(time::span deltaTime)
    //{
    //    static auto physicsQuery = createQuery< physics::physicsComponent>();
    //    uint i = 0;

    //    float duration = 0.02f;

    //    for (auto penetration : physics::PhysicsSystem::penetrationQueries)
    //    {
    //        debug::drawLine(penetration->faceCentroid
    //            , penetration->faceCentroid + penetration->normal, math::vec4(1, 0, 1, 1), 15.0f, duration);
    //        auto x = 1;
    //    }


    //    //--------------------------------------- Draw contact points ---------------------------------------//



    //    for (int i = 0; i < physics::PhysicsSystem::contactPoints.size(); i++)
    //    {
    //        //ref is red
    //        //inc is blue

    //        auto& contact = physics::PhysicsSystem::contactPoints.at(i);

    //        debug::drawLine(contact.refRBCentroid
    //            , contact.RefWorldContact, math::vec4(1, 0, 0, 1), 5.0f, duration, true);

    //        debug::drawLine(contact.incRBCentroid
    //            , contact.IncWorldContact, math::vec4(0, 0, 1, 1), 5.0f, duration, true);

    //        debug::drawLine(contact.IncWorldContact
    //            , contact.IncWorldContact + math::vec3(0, 0.1f, 0), math::vec4(0.5, 0.5, 0.5, 1), 5.0f, duration, true);

    //        debug::drawLine(contact.refRBCentroid
    //            , contact.refRBCentroid + math::vec3(0, 0.1f, 0), math::vec4(0, 0, 0, 1), 5.0f, duration, true);

    //    }

    //    //--------------------------------------- Draw extreme points ---------------------------------------//

    //    i = 0;
    //    for (auto penetration : physics::PhysicsSystem::aPoint)
    //    {
    //        debug::drawLine(penetration
    //            , penetration + math::vec3(0, 0.2, 0), math::vec4(1, 0, 0, 1), 15.0f);

    //    }
    //    i = 0;
    //    for (auto penetration : physics::PhysicsSystem::bPoint)
    //    {
    //        debug::drawLine(penetration
    //            , penetration + math::vec3(0, 0.2, 0), math::vec4(0, 0, 1, 1), 15.0f);

    //    }

    //    physics::PhysicsSystem::contactPoints.clear();
    //    physics::PhysicsSystem::penetrationQueries.clear();
    //    physics::PhysicsSystem::aPoint.clear();
    //    physics::PhysicsSystem::bPoint.clear();

    //    //physicsQuery.queryEntities();
    //    //auto size = physicsQuery.size();
    //    ////this is called so that i can draw stuff
    //    //for (auto entity : physicsQuery)
    //    //{
    //    //    auto rotationHandle = entity.get_component_handle<rotation>();
    //    //    auto positionHandle = entity.get_component_handle<position>();
    //    //    auto scaleHandle = entity.get_component_handle<scale>();
    //    //    auto physicsComponentHandle = entity.get_component_handle<physics::physicsComponent>();

    //    //    bool hasTransform = rotationHandle && positionHandle && scaleHandle;
    //    //    bool hasNecessaryComponentsForPhysicsManifold = hasTransform && physicsComponentHandle;

    //    //    if (hasNecessaryComponentsForPhysicsManifold)
    //    //    {
    //    //        auto rbColor = math::color(0.0, 0.5, 0, 1);
    //    //        auto statibBlockColor = math::color(0, 1, 0, 1);

    //    //        rotation rot = rotationHandle.read();
    //    //        position pos = positionHandle.read();
    //    //        scale scale = scaleHandle.read();

    //    //        auto usedColor = statibBlockColor;
    //    //        bool useDepth = false;

    //    //        if (entity.get_component_handle<physics::rigidbody>())
    //    //        {
    //    //            usedColor = rbColor;
    //    //        }


    //    //        //assemble the local transform matrix of the entity
    //    //        math::mat4 localTransform;
    //    //        math::compose(localTransform, scale, rot, pos);

    //    //        auto physicsComponent = physicsComponentHandle.read();

    //    //        i = 0;
    //    //        for (auto physCollider : *physicsComponent.colliders)
    //    //        {
    //    //            //--------------------------------- Draw Collider Outlines ---------------------------------------------//

    //    //            for (auto face : physCollider->GetHalfEdgeFaces())
    //    //            {
    //    //                //face->forEachEdge(drawFunc);
    //    //                physics::HalfEdgeEdge* initialEdge = face->startEdge;
    //    //                physics::HalfEdgeEdge* currentEdge = face->startEdge;

    //    //                math::vec3 faceStart = localTransform * math::vec4(face->centroid, 1);
    //    //                math::vec3 faceEnd = faceStart + math::vec3((localTransform * math::vec4(face->normal, 0)));

    //    //                //debug::drawLine(faceStart, faceEnd, math::colors::green, 5.0f);

    //    //                if (!currentEdge) { return; }

    //    //                do
    //    //                {
    //    //                    physics::HalfEdgeEdge* edgeToExecuteOn = currentEdge;
    //    //                    currentEdge = currentEdge->nextEdge;

    //    //                    math::vec3 worldStart = localTransform * math::vec4(edgeToExecuteOn->edgePosition, 1);
    //    //                    math::vec3 worldEnd = localTransform * math::vec4(edgeToExecuteOn->nextEdge->edgePosition, 1);

    //    //                    debug::drawLine(worldStart, worldEnd, usedColor, 2.0f, 0.0f, useDepth);

    //    //                } while (initialEdge != currentEdge && currentEdge != nullptr);
    //    //            }
    //    //        }

    //    //    }

    //    //}

    //    //FindClosestPointsToLineSegment unit test


    //   /* math::vec3 p1(5, -0.5, 0);
    //    math::vec3 p2(5, 0.5, 0);

    //    math::vec3 p3(6, 0, -0.5);
    //    math::vec3 p4(6, 0, 0.5);

    //    math::vec3 p1p2;
    //    math::vec3 p3p4;

    //    debug::drawLine(p1, p2, math::colors::red, 5.0f);
    //    debug::drawLine(p3, p4, math::colors::red, 5.0f);

    //    physics::PhysicsStatics::FindClosestPointsToLineSegment(p1, p2, p3, p4, p1p2, p3p4);

    //    debug::drawLine(p1p2, p3p4, math::colors::green, 5.0f);

    //    p1 = math::vec3(8, 0, 0);
    //    p2 = p1 + math::vec3(0, 1.0f, 0);

    //    p3 = math::vec3(10, 0, 0) + math::vec3(1.0f);
    //    p4 = p3 - math::vec3(1.0f);

    //    debug::drawLine(p1, p2, math::colors::red, 5.0f);
    //    debug::drawLine(p3, p4, math::colors::red, 5.0f);

    //    physics::PhysicsStatics::FindClosestPointsToLineSegment(p1, p2, p3, p4, p1p2, p3p4);

    //    debug::drawLine(p1p2, p3p4, math::colors::green, 5.0f);*/

    //}

    void onActivateUnitTest2(activate_CRtest2* action)
    {
        static bool activated = false;

        if (action->value)
        {
            activated = true;
            auto crb = m_ecs->createComponent<physics::rigidbody>(staticToOBBEnt);
        }
    }

    void onActivateUnitTest3(activate_CRtest3* action)
    {
        if (action->value)
        {
            auto crb = m_ecs->createComponent<physics::rigidbody>(staticToEdgeEnt);
        }
    }

    //void onExtendedPhysicsContinueRequest(extendedPhysicsContinue* action)
    //{
    //    if (action->value)
    //    {
    //        physics::PhysicsSystem::IsPaused = false;
    //    }
    //    else
    //    {
    //        physics::PhysicsSystem::IsPaused = true;
    //    }

    //}

    //void onNextPhysicsTimeStepRequest(nextPhysicsTimeStepContinue* action)
    //{
    //    if (!(action->value))
    //    {
    //        physics::PhysicsSystem::IsPaused = true;
    //        physics::PhysicsSystem::oneTimeRunActive = true;
    //        log::debug(" onNextPhysicsTimeStepRequest");
    //    }

    //}


    /*void FrictionTestActivate(activateFrictionTest* action)
    {
        if (action->value)
        {
            rotate = true;
            {
                auto rbH = m_ecs->createComponent<physics::rigidbody>(NoFrictionBody);

                auto rb = rbH.read();

                rb.friction = 0.0f;

                rbH.write(rb);
            }

            {
                auto rbH = m_ecs->createComponent<physics::rigidbody>(Point3FrictionBody);

                auto rb = rbH.read();

                rb.friction = 0.1f;

                rbH.write(rb);
            }

            {
                auto rbH = m_ecs->createComponent<physics::rigidbody>(Point6FrictionBody);

                auto rb = rbH.read();

                rb.friction = 0.4f;

                rbH.write(rb);
            }

            {
                auto rbH = m_ecs->createComponent<physics::rigidbody>(FullFrictionBody);

                auto rb = rbH.read();

                rb.friction = 1.0f;

                rbH.write(rb);
            }


        }
    }*/

    void CreateCubeStack(int height, int width, int breadth, math::vec3 startPosition, math::vec3 offset,
        physics::cube_collider_params cubeParams, float cubeFriction, rendering::model_handle cubeH, rendering::material_handle wireframeH, bool addRigidbody = true)
    {
        int l = 0;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                for (int k = 0; k < breadth; k++)
                {
                    auto ent = m_ecs->createEntity();

                    auto entPhyHande = ent.add_component<physics::physicsComponent>();

                    //auto renderableHandle = m_ecs->createComponents<rendering::mesh_renderable>(ent);
                    //renderableHandle.write({ cubeH, wireframeH });
                    ent.add_components<rendering::mesh_renderable>(mesh_filter(cubeH.get_mesh()), rendering::mesh_renderer(wireframeH));

                    physics::physicsComponent physicsComponent;
                    physics::physicsComponent::init(physicsComponent);
                    physicsComponent.AddBox(cubeParams);
                    physicsComponent.isTrigger = false;
                    entPhyHande.write(physicsComponent);

                    auto [positionH, rotationH, scaleH] = m_ecs->createComponents<transform>(ent);
                    positionH.write(startPosition + math::vec3(j * offset.x, i * offset.y, k * offset.z));
                    scaleH.write(math::vec3(1.0f));

                    if (addRigidbody)
                    {
                        auto rbH = m_ecs->createComponent<physics::rigidbody>(ent);

                        auto rb = rbH.read();

                        rb.friction = cubeFriction;

                        rbH.write(rb);

                    }

                    auto idHandle = m_ecs->createComponent<physics::identifier>(ent);
                    auto idComp = idHandle.read();
                    idComp.id = "Stack" + std::to_string(l);
                    l++;
                    idHandle.write(idComp);

                }
            }
        }
    }

    void OnNextEdge(nextEdge_action* action)
    {
        /*static ecs::EntityQuery halfEdgeQuery = createQuery<physics::MeshSplitter>();

        if (action->value)
        {
            log::debug("OnNextEdge(nextEdge_action* action)");
            for (auto entity : halfEdgeQuery)
            {

                auto edgeFinderH = entity.get_component_handle<physics::MeshSplitter>();

                auto splitter = edgeFinderH.read();


                splitter .edgeFinder.currentPtr = splitter .edgeFinder.currentPtr->nextEdge;

                edgeFinderH.write(splitter);

            }
        }*/


    }

    void OnNextPair(nextPairing_action* action)
    {
        /*static ecs::EntityQuery halfEdgeQuery = createQuery<physics::MeshSplitter>();

        if (action->value)
        {
            log::debug("OnNextPair(nextPairing_action * action)");
            for (auto entity : halfEdgeQuery)
            {

                auto edgeFinderH = entity.get_component_handle<physics::MeshSplitter>();

                auto splitter = edgeFinderH.read();

                splitter .edgeFinder.currentPtr = splitter .edgeFinder.currentPtr->pairingEdge;


                edgeFinderH.write(splitter);

            }
        }*/

    }





};
