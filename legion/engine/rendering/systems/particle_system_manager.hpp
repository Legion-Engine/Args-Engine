#pragma once
#include <core/core.hpp>
#include <rendering/data/particle_system_base.hpp>
#include <rendering/components/point_emitter_data.hpp>
namespace legion::rendering
{
    /**
     * @class ParticleSystemManager
     * @brief The class used to update all particles in every emitter.
     */
    class ParticleSystemManager : public System<ParticleSystemManager>
    {
    public:
        ParticleSystemManager()
        {
            ParticleSystemBase::m_registry = m_ecs;
        }
        /**
         * @brief Sets up the particle system manager.
         */
        void setup()
        {
            createProcess<&ParticleSystemManager::update>("Update");
        }
        /**
         * @brief Every frame, goes through every emitter and updates their particles with their respective particle systems.
         * @param deltaTime The delta time to be used inside of the update.
         */
        void update(time::span deltaTime)
        {
            OPTICK_EVENT();
            static auto emitters = createQuery<particle_emitter>();
            emitters.queryEntities();
            for (auto entity : emitters)
            {
                //Gets emitter handle and emitter.
                auto emitterHandle = entity.get_component_handle<particle_emitter>();
                auto emit = emitterHandle.read();
                //Checks if emitter was already initialized.
                if (!emit.setupCompleted)
                {
                    //If NOT then it goes through the particle system setup.
                    emit.setupCompleted = true;
                    emitterHandle.write(emit);

                    const ParticleSystemBase* particleSystem = emit.particleSystemHandle.get();
                    particleSystem->setup(emitterHandle);
                }
                else
                {
                    //If it IS then it runs the emitter through the particle system update.
                    const ParticleSystemBase* particleSystem = emit.particleSystemHandle.get();
                    particleSystem->update(emit.livingParticles, emitterHandle, emitters, deltaTime);
                }
            }



            //update point cloud buffer data
            static auto pointCloudQuery = createQuery<particle_emitter, rendering::point_emitter_data>();
            pointCloudQuery.queryEntities();
            std::vector<math::vec4> colorData;
            int index = 0;
            for (auto pointEntities : pointCloudQuery)
            {
                auto emitterHandle = pointEntities.get_component_handle<particle_emitter>();
                auto emitter = emitterHandle.read();
                const ParticleSystemBase* particleSystem = emitter.particleSystemHandle.get();

                auto dataHandle = pointEntities.get_component_handle<rendering::point_emitter_data>();
                auto data = dataHandle.read();

                index++;
                if (index == pointCloudQuery.size())
                {
                    auto window = ecs::EcsRegistry::world.read_component<app::window>();
                    app::context_guard guard(window);
                    if (guard.contextIsValid())
                    {
                        ////create buffer
                        rendering::buffer colorBuffer = rendering::buffer(GL_ARRAY_BUFFER, emitter.container->colorBufferData, GL_STREAM_DRAW);
                        particleSystem->m_particleModel.overwrite_buffer(colorBuffer, SV_COLOR, true);
                        log::debug(std::to_string(emitter.container->colorBufferData.size()));

                    }
                }
            }
        }
    };
}
