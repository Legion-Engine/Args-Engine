#pragma once
#include <core/core.hpp>
#include <rendering/components/particle.hpp>
#include <rendering/components/particle_emitter.hpp>
#include <rendering/data/material.hpp>
#include <rendering/data/model.hpp>

namespace legion::rendering
{
    /**
     * @class ParticleSystemBase
     * @brief Base class to be used as a template for the creation of particle systems.
     */
    class ParticleSystemBase
    {
        friend class ParticleSystemManager;
    public:
        virtual ~ParticleSystemBase() = default;


        /**
         * @brief The function that is run to setup all the particles inside of the given emitter.
         * @param particle_emitter The particle emitter that holds the particles that you plan to iterate over.
         */
        virtual void setup(ecs::component_handle<particle_emitter> particle_emitter) const LEGION_IMPURE;
        /**
         * @brief The function that runs every frame to update all the particles inside of the given emitter.
         * @param particle_list A list of particles to iterate over.
         * @param particle_emitter The emitter component handle holding the particles.
         */
        virtual void update(std::vector<ecs::entity_handle>& particle_list, ecs::component_handle<particle_emitter> particle_emitter, ecs::EntityQuery& entities, time::span delta_time) const LEGION_IMPURE;

    protected:
        /**
         * @brief The function used to populate particles with data.
         * @param particleHandle The particle that you want to populate.
         * @param transformHandle The transform component of the particle.
         */
        virtual void createParticle(
            ecs::component_handle<transform> transformHandle) const;
        /**
         * @brief The function used to clean up particles that have outlived their lifeTime.
         * @param particleHandle The particle that is to be cleaned up.
         * @param emitterHandle The emitter that hold that particular particle.
         */
        virtual void cleanUpParticle(
            ecs::entity_handle particleHandle,
            particle_emitter& emitterHandle) const;
        /**
         * @brief Checks if the given emitter has a particle ready to be used, otherwise make a new one.
         * @param emitterHandle The emitter that is being checked for an available particle.
         */
        ecs::component_handle<particle> checkToRecycle(
            particle_emitter& emitterHandle) const;

        bool m_looping;

        float m_maxLifeTime;
        float m_startingLifeTime;

        uint m_spawnRate;
        uint m_particleCount;
        uint m_maxParticles;

        math::vec3 m_startingVelocity;

        math::vec3 m_startingSize;
        float m_sizeOverLifetime;

        material_handle m_particleMaterial;
        model_handle m_particleModel;
        static ecs::EcsRegistry* m_registry;
    };
}
