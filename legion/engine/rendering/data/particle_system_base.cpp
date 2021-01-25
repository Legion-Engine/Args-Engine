#include <rendering/data/particle_system_base.hpp>
#include <rendering/components/particle_emitter.hpp>
#include <rendering/components/renderable.hpp>


namespace legion::rendering
{
    ecs::EcsRegistry* ParticleSystemBase::m_registry;

    void ParticleSystemBase::createParticle(ecs::component_handle<transform> transformHandle) const
    {
        OPTICK_EVENT();
         ecs::entity_handle entity = transformHandle.entity;

         //Handle model and material assigning.
         entity.add_component<mesh_renderer>(rendering::mesh_renderer(m_particleMaterial, m_particleModel));
    }

    void ParticleSystemBase::cleanUpParticle(ecs::entity_handle particleHandle, particle_emitter& emitter) const
    {
        OPTICK_EVENT();
        //Read emitter
        auto it = std::find(emitter.livingParticles.begin(), emitter.livingParticles.end(), particleHandle);
        if (it != emitter.livingParticles.end())
        {
            //Remove from living
            emitter.livingParticles.erase(it);
            //Add to dead
            emitter.deadParticles.emplace_back(particleHandle);
            //Remove renderable to stop them from being rendered
            particleHandle.remove_component<mesh_renderer>();
        }
    }

    ecs::component_handle<particle> ParticleSystemBase::checkToRecycle(rendering::particle_emitter& emitter) const
    {
        ecs::entity_handle particularParticle;

        if (!emitter.deadParticles.empty())
        {
            //Get particle from dead particle list.
            particularParticle = emitter.deadParticles.back();
            //remove last item
            emitter.deadParticles.pop_back();
        }
        else
        {
            //Create new particle entity.
            particularParticle = m_registry->createEntity();
            //give newly created particle a transform
            particularParticle.add_components<transform>();
            particularParticle.add_component<particle>();
        }
        //Add particle to living particle list.
        emitter.livingParticles.push_back(particularParticle);

        return particularParticle.get_component_handle<particle>();
    }
}
