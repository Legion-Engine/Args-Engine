#pragma once

#include <core/core.hpp>
#include <physics/systems/physicssystem.hpp>
#include <physics/components/physics_component.hpp>
#include <physics/components/rigidbody.hpp>
#include <physics/mesh_splitter_utils/mesh_splitter.hpp>
#include <physics/components/fracturer.hpp>

namespace legion::physics
{
    class PhysicsModule : public Module
    {
    public:

        virtual void setup() override
        {
            addProcessChain("Physics");
            reportSystem<PhysicsSystem>();
            reportComponentType<physicsComponent>();
            reportComponentType<rigidbody>();
            reportComponentType<identifier>();
            reportComponentType<MeshSplitter>();
            reportComponentType<Fracturer>();
            //reportComponentType <addRB>();
        }

        virtual priority_type priority() override
        {
            return 20;
        }

    };

}


