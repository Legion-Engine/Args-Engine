#pragma once
#include <core/core.hpp>
#include <rendering/data/particle_system_cache.hpp>
#include<rendering/components/point_cloud_particle_container.hpp>
namespace legion::rendering
{
    /**
     * @brief Particle Emitter is the component that hold all the particles and related particle system.
     */
    struct particle_emitter
    {
        std::vector<ecs::entity_handle> livingParticles;
        std::vector<ecs::entity_handle> deadParticles;
        bool playAnimation = false;
        ParticleSystemHandle particleSystemHandle;
        bool setupCompleted;

        std::vector<math::vec3> pointInput;
        std::vector<math::vec4> colorInput;
        point_cloud_particle_container* container;
    };


}
