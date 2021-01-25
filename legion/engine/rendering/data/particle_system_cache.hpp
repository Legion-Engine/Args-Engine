#pragma once
#include <core/core.hpp>

namespace legion::rendering
{
    class ParticleSystemBase;
    /**
     * @struct ParticleSystemHandle
     * @brief The handle for a particle system.
     */
    struct ParticleSystemHandle
    {
        id_type id;
        const ParticleSystemBase* get() const;
        bool validate() const noexcept;
    };

    /**
     * @class ParticleSystemCache
     * @brief The cache class the holds all the particle systems.
     */
    class ParticleSystemCache
    {
        friend struct ParticleSystemHandle;
    public:
        //TODO(Algo-Ryth-Mix) fix static assert!
        /** @brief Creates a particle system and returns it.
         *  @tparam T A particle system that inherits from ParticleSystembase.
         *  @param name The name of the particle system.
         *  @params args Additionally forwarded arguments for the creation of the particle system.
         */
        template<class T, class ... Args>
        static ParticleSystemHandle createParticleSystem(std::string name, Args&&...args)
        {
            //static_assert(std::is_convertible<ParticleSystemBase*, T*>::value, "T must inherit from ParticleSystemBase!");
            return createParticleSystemImpl(std::move(name), new T(args...));
        }
        /**
         * @brief Gets the particle system with the given name.
         * @param name The name of the wanted particle system.
         */
        static ParticleSystemHandle getParticleSystem(std::string name)
        {
            const auto id = nameHash(name);
            async::readonly_guard guard(m_particleSystemLock);
            if (m_cache.find(id) == m_cache.end()) return ParticleSystemHandle{ invalid_id };
            return ParticleSystemHandle{ id };
        }
    private:
        /**
         * @brief Gets the particle system pointer wit the given id.
         * @param id The id of the wanted particle system pointer.
         */
        static const ParticleSystemBase* getParticleSystemPointer(id_type id)
        {
            const auto iterator = m_cache.find(id);
            if (iterator == m_cache.end()) return nullptr;
            return iterator->second.get();
        }
        static ParticleSystemHandle createParticleSystemImpl(const std::string& name, ParticleSystemBase* system);

        static std::unordered_map<id_type, std::unique_ptr<const ParticleSystemBase>> m_cache;
        static async::rw_spinlock m_particleSystemLock;
    };

    inline const ParticleSystemBase* ParticleSystemHandle::get() const
    {
        return ParticleSystemCache::getParticleSystemPointer(id);
    }

    inline bool ParticleSystemHandle::validate() const noexcept
    {
        if (ParticleSystemCache::getParticleSystemPointer(id) == nullptr) return false;
        return true;
    }
}

