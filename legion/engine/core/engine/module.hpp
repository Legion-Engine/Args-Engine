#pragma once
#include <core/types/primitives.hpp>
#include <core/types/meta.hpp>
#include <core/platform/platform.hpp>
#include <core/engine/system.hpp>
#include <core/containers/sparse_map.hpp>
#include <core/ecs/registry.hpp>
#include <core/events/eventbus.hpp>
#include <memory>

/**
 * @file module.hpp
 */

namespace legion::core
{
    /**@class Module
     * @brief interface for engine-modules, must be implemented
     * @ref legion::core::Engine::reportModule<T,...>()
     */
    class Module
    {
        friend class Engine;
    private:
        multicast_delegate<void()> m_setupFuncs;
        std::vector<delegate<void(time::span)>> m_updateFuncs;

        sparse_map<id_type, std::unique_ptr<SystemBase>> m_systems;

        void init()
        {
            m_setupFuncs.invoke();
            for (auto& updateFunc : m_updateFuncs)
            {
                // Create update process.
            }
        };

    protected:
        template<size_type charc>
        void addProcessChain(const char(&name)[charc])
        {
            //m_scheduler->addProcessChain<charc>(name);
        }

        template<typename SystemType, typename... Args CNDOXY(inherits_from<SystemType, System<SystemType>> = 0)>
        void reportSystem(Args&&... args)
        {
            m_systems.insert(make_hash<SystemType>(), std::make_unique<SystemType>(std::forward<Args>(args)...));
            if constexpr (has_setup_v<SystemType, void()>)
            {
                m_setupFuncs += delegate<void()>::create<SystemType, &SystemType::setup>(static_cast<SystemType*>(m_systems.at(make_hash<SystemType>()).get()));
            }
            if constexpr (has_update_v<SystemType, void(time::span)>)
            {
                m_updateFuncs.push_back(delegate<void()>::create<SystemType, &SystemType::setup>(static_cast<SystemType*>(m_systems.at(make_hash<SystemType>()).get())));
            }
        }

    public:
        virtual void setup() LEGION_PURE;

        /**@brief determines the execution priority of this module
         * @ref LEGION_IMPURE_RETURN
         * @returns priority_type signed int8 higher is higher priority and get called first
         * @note default priority of the engine is 0.
         * @note call order for modules with the same priority is undefined.
         */
        virtual priority_type priority() LEGION_IMPURE_RETURN(default_priority);

        virtual ~Module() = default;
    };
}
