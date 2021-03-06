#pragma once
#include <core/core.hpp>
#include <core/math/math.hpp>
#include "../systems/testsystem.hpp"

//#include "../systems/testsystem2.hpp"
#include "../systems/simplecameracontroller.hpp"
#include "../systems/gui_test.hpp"

#include <physics/systems/physics_fracture_test_system.hpp>


#include "../data/animation.hpp"
#include "../systems/animation_editor.hpp"
#include "../systems/animator.hpp"
#include "../systems/event_based_explosion_system.hpp"
#include "../systems/testsystemconvexhull.hpp"
#include "../systems/final_presentation_system.hpp"


using namespace legion;

class TestModule : public Module
{
public:
    virtual void setup() override
    {

        // Create the request for the main window.
        app::WindowSystem::requestWindow(
            world_entity_id,
            math::ivec2(1920, 1080),
            "LEGION Engine",
            "Legion Icon",
            nullptr,
            nullptr,
            1
        );

        reportComponentType<sah>();
        reportSystem<TestSystem>();
        reportSystem<SimpleCameraController>();
        //reportSystem<pointcloudtestsystem2>();
        reportComponentType<ext::animation>();
        reportComponentType<ext::evt::particle_receiver>();
        reportComponentType<ext::evt::explosion_receiver>();
        reportComponentType<ext::evt::decal_receiver>();
        //reportSystem<ext::Animator>();
        //reportSystem<ext::AnimationEditor>();
        //reportSystem<final_presentation_system>();
        reportSystem<GuiTestSystem>();
        //reportSystem<physics::PhysicsFractureTestSystem>();
        reportSystem<TestSystemConvexHull>();
        //reportSystem<ext::BigBoom>();

    }

    virtual priority_type priority() override
    {
        return default_priority;
    }
};

