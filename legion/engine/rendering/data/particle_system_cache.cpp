#include <rendering/data/particle_system_cache.hpp>
#include <rendering/data/particle_system_base.hpp>

namespace legion::rendering
{
    std::unordered_map<id_type, std::unique_ptr<const ParticleSystemBase>> ParticleSystemCache::m_cache;
    async::rw_spinlock ParticleSystemCache::m_particleSystemLock;

    ParticleSystemHandle ParticleSystemCache::createParticleSystemImpl(const std::string& name, ParticleSystemBase* system)
    {
        id_type id = nameHash(name);

        auto got = m_cache.find(id);
        if (got == m_cache.end())
        {
            //acquire lock
            async::readwrite_guard guard(m_particleSystemLock);

            //emplace kv-pair
            auto iterator = m_cache.emplace(id, system);

            //check for success
            if (iterator.second)
                return ParticleSystemHandle{ id };
            return ParticleSystemHandle{ invalid_id };
        }
        return ParticleSystemHandle{ id };
    }

}
