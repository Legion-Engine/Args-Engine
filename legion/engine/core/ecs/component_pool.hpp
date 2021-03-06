#pragma once
#include <core/async/rw_spinlock.hpp>
#include <core/async/transferable_atomic.hpp>
#include <core/platform/platform.hpp>
#include <core/containers/atomic_sparse_map.hpp>
#include <core/types/types.hpp>
#include <core/events/eventbus.hpp>
#include <core/events/events.hpp>
#include <core/ecs/component_meta.hpp>
#include <core/ecs/component_container.hpp>

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>

#include <core/serialization/serializationmeta.hpp>

#include <functional>

#include <Optick/optick.h>

/**
 * @file component_pool.hpp
 */

namespace legion::core::ecs
{
    class EcsRegistry;

    using entity_container = std::vector<entity_handle>;

    /**@class component_pool_base
     * @brief Base class of legion::core::ecs::component_pool
     */
    class component_pool_base
    {
    public:
        virtual component_container_base* get_components(const entity_container& entities) const LEGION_PURE;
        virtual void get_components(const entity_container& entities, component_container_base& comps) const LEGION_PURE;
        virtual void set_components(const entity_container& entities, const component_container_base& comps) LEGION_PURE;

        L_NODISCARD virtual bool has_component(id_type entityId) const LEGION_PURE;
        virtual void create_component(id_type entityId) LEGION_PURE;
        virtual void create_component(id_type entityId, void* value) LEGION_PURE;
        virtual void destroy_component(id_type entityId) LEGION_PURE;

        virtual void clone_component(id_type dst, id_type src) LEGION_PURE;

        virtual void serialize(cereal::JSONOutputArchive& oarchive, id_type entityId) LEGION_PURE;
        virtual void serialize(cereal::BinaryOutputArchive& oarchive, id_type entityId) LEGION_PURE;

        virtual void serialize(cereal::JSONInputArchive& oarchive, id_type entityId) LEGION_PURE;
        virtual void serialize(cereal::BinaryInputArchive& oarchive, id_type entityId) LEGION_PURE;

        virtual ~component_pool_base() = default;
    };

    /**@class component_pool
     * @brief Thread-safe container to store a component family in.
     * @tparam component_type Type of component.
     */
    template<typename component_type>
    class component_pool : public component_pool_base
    {
    private:
        sparse_map<id_type, component_type> m_components;
        mutable async::rw_spinlock m_lock;

        events::EventBus* m_eventBus;
        EcsRegistry* m_registry;
        component_type m_nullComp;
    public:
        component_pool() = default;
        component_pool(EcsRegistry* registry, events::EventBus* eventBus) : m_registry(registry), m_eventBus(eventBus) {}

        void serialize(cereal::JSONOutputArchive& oarchive, id_type entityId) override
        {
            static_assert(!(serialization::has_serialize<component_type, void(cereal::JSONOutputArchive&)>::value &&
                            serialization::has_load<component_type, void(cereal::JSONInputArchive&)>::value),
                            "Serialized Objects should not have load&save pairs and a serialization function simultaneously");

            OPTICK_EVENT();
            std::string componentType = std::string(nameOfType<component_type>());

            if constexpr (serialization::has_serialize<component_type, void(cereal::JSONOutputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                oarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].serialize(oarchive);
            }
            else if constexpr (serialization::has_save<component_type, void(cereal::JSONOutputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                oarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].save(oarchive);
            }
            else
            {
                oarchive(cereal::make_nvp("Component Name", componentType));
            }
        }
        void serialize(cereal::BinaryOutputArchive& oarchive, id_type entityId) override
        {
            static_assert(!(serialization::has_serialize<component_type, void(cereal::BinaryOutputArchive&)>::value &&
                            serialization::has_load<component_type, void(cereal::BinaryOutputArchive&)>::value),
                            "Serialized Objects should not have load&save pairs and a serialization function simultaneously");

            OPTICK_EVENT();
            std::string componentType = std::string(nameOfType<component_type>());

            if constexpr (serialization::has_serialize<component_type, void(cereal::BinaryOutputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                oarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].serialize(oarchive);
            }
            else if constexpr (serialization::has_save<component_type, void(cereal::BinaryOutputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                oarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].save(oarchive);
            }
            else
            {
                oarchive(cereal::make_nvp("Component Name", componentType));
            }
        }

        void serialize(cereal::JSONInputArchive& iarchive, id_type entityId) override
        {
            static_assert(!(serialization::has_serialize<component_type, void(cereal::JSONOutputArchive&)>::value &&
                            serialization::has_load<component_type, void(cereal::JSONInputArchive&)>::value),
                            "Serialized Objects should not have load&save pairs and a serialization function simultaneously");

            OPTICK_EVENT();

            std::string componentType = std::string(nameOfType<component_type>());
            if constexpr (serialization::has_serialize<component_type, void(cereal::JSONOutputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                iarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].serialize(iarchive);
            }
            else if constexpr (serialization::has_load<component_type, void(cereal::JSONInputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                iarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].load(iarchive);
            }
            else
            {
                iarchive(cereal::make_nvp("Component Name", componentType));
            }
        }

        void serialize(cereal::BinaryInputArchive& iarchive, id_type entityId) override
        {
            static_assert(!(serialization::has_serialize<component_type, void(cereal::BinaryInputArchive&)>::value &&
                            serialization::has_load<component_type, void(cereal::BinaryInputArchive&)>::value),
                            "Serialized Objects should not have load&save pairs and a serialization function simultaneously");

            OPTICK_EVENT();

            std::string componentType = std::string(nameOfType<component_type>());
            if constexpr (serialization::has_serialize<component_type, void(cereal::BinaryInputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                iarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].serialize(iarchive);
            }
            else if constexpr (serialization::has_load<component_type, void(cereal::BinaryInputArchive&)>::value)
            {
                async::readonly_guard guard(m_lock);
                iarchive(cereal::make_nvp("Component Name", componentType));
                m_components[entityId].load(iarchive);
            }
            else
            {
                iarchive(cereal::make_nvp("Component Name", componentType));
            }
        }

        /**@brief Get the rw_spinlock of this container.
         */
        async::rw_spinlock& get_lock() const noexcept
        {
            return m_lock;
        }

        component_container_base* get_components(const entity_container& entities) const override
        {
            OPTICK_EVENT();
            auto* container = new component_container<component_type>();
            container->resize(entities.size());

            async::readonly_guard guard(m_lock);
            for (int i = 0; i < entities.size(); i++)
            {
                OPTICK_EVENT("Get component");
#ifdef LGN_SAFE_MODE
                if (m_components.contains(entities[i]))
                    container->at(i) = m_components.at(entities[i]);
#else
                container->at(i) = m_components.at(entities[i]);
#endif
            }

            return container;
        }


        void get_components(const entity_container& entities, component_container_base& comps) const override
        {
            OPTICK_EVENT();
            component_container<component_type>& container = comps.cast<component_type>();
            container.resize(entities.size());

#ifdef LGN_SAFE_MODE
            if (!container.getComponentTypeId())
                return;
#endif

            async::readonly_guard guard(m_lock);
            for (int i = 0; i < entities.size(); i++)
            {
                OPTICK_EVENT("Get component");
#ifdef LGN_SAFE_MODE
                if (m_components.contains(entities[i]))
                    container[i] = m_components.at(entities[i]);
#else
                container[i] = m_components.at(entities[i]);
#endif
            }
        }

        void set_components(const entity_container& entities, const component_container_base& comps) override
        {
            OPTICK_EVENT();
            const component_container<component_type>& container = comps.cast<component_type>();

#ifdef LGN_SAFE_MODE
            if (!container.getComponentTypeId())
                return;
#endif

            component_container<component_type> modifications;
            modifications.resize(entities.size());

            {
                async::readonly_guard guard(m_lock);
                for (int i = 0; i < entities.size(); i++)
                {
                    auto& ent = entities[i];
                    if (m_components.contains(ent))
                    {
                        component_type& ref = m_components.at(ent);
                        modifications[i] = ref;
                        ref = container[i];
                    }
                }
            }

            m_eventBus->raiseEvent<events::bulk_component_modification<component_type>>(entities, modifications, container);
        }

        /**@brief Thread-safe check for whether an entity has the component.
         * @param entityId ID of the entity you wish to check for.
         */
        L_NODISCARD bool has_component(id_type entityId) const override
        {
            OPTICK_EVENT();
            async::readonly_guard guard(m_lock);
            return m_components.contains(entityId);
        }

        /**@brief Thread unsafe component fetch, use component_pool::get_lock and lock for at least read_only before calling this function.
         * @param entityId ID of entity you want to get the component from.
         * @ref component_pool::get_lock()
         * @ref legion::core::async::rw_spinlock
         */
        L_NODISCARD component_type& get_component(id_type entityId)
        {
            OPTICK_EVENT();
            if (m_components.contains(entityId))
                return m_components.at(entityId);
            return m_nullComp;
        }

        /**@brief Thread unsafe component fetch, use component_pool::get_lock and lock for at least read_only before calling this function.
         * @param entityId ID of entity you want to get the component from.
         * @ref component_pool::get_lock()
         * @ref legion::core::async::rw_spinlock
         */
        L_NODISCARD const component_type& get_component(id_type entityId) const
        {
            OPTICK_EVENT();
            if (m_components.contains(entityId))
                return m_components.at(entityId);
            return m_nullComp;
        }

        /**@brief Creates component in a thread-safe way.
         * @note Calls component_type::init if it exists.
         * @note Raises the events::component_creation<component_type>> event.
         * @param entityId ID of entity you wish to add the component to.
         */
        void create_component(id_type entityId) override
        {
            OPTICK_EVENT();
            {
                async::readwrite_guard guard(m_lock);
                m_components.emplace(entityId);
            }

            if constexpr (detail::has_init<component_type, void(component_type&, entity_handle)>::value)
            {
                async::readonly_guard rguard(m_lock);
                component_type::init(m_components[entityId], entity_handle(entityId));
            }
            else if constexpr (detail::has_init<component_type, void(component_type&)>::value)
            {
                async::readonly_guard rguard(m_lock);
                component_type::init(m_components[entityId]);
            }

            m_eventBus->raiseEvent<events::component_creation<component_type>>(entity_handle(entityId));
        }

        /**@brief Creates component in a thread-safe way and initializes it with the given value.
         * @note Does NOT call component_type::init.
         * @note Raises the events::component_creation<component_type>> event.
         * @param entityId ID of entity you wish to add the component to.
         * @param value Pointer to component_type that has the starting value you require.
         */
        void create_component(id_type entityId, void* value) override
        {
            OPTICK_EVENT();
            {
                async::readwrite_guard guard(m_lock);
                m_components[entityId] = *reinterpret_cast<component_type*>(value);
            }

            if constexpr (detail::has_init<component_type, void(component_type&, entity_handle)>::value)
            {
                async::readonly_guard rguard(m_lock);
                component_type::init(m_components[entityId], entity_handle(entityId));
            }
            else if constexpr (detail::has_init<component_type, void(component_type&)>::value)
            {
                async::readonly_guard rguard(m_lock);
                component_type::init(m_components[entityId]);
            }

            m_eventBus->raiseEvent<events::component_creation<component_type>>(entity_handle(entityId));
        }


        /**@brief Destroys component in a thread-safe way.
         * @note Calls component_type::destroy if it exists.
         * @note Raises the events::component_destruction<component_type>> event.
         * @param entityId ID of entity you wish to remove the component from.
         */
        void destroy_component(id_type entityId) override
        {
            OPTICK_EVENT();
            m_eventBus->raiseEvent<events::component_destruction<component_type>>(entity_handle(entityId));

            if constexpr (detail::has_destroy<component_type, void(component_type&)>::value)
            {
                async::readonly_guard rguard(m_lock);
                component_type::destroy(m_components[entityId]);
            }

            async::readwrite_guard wguard(m_lock);
            m_components.erase(entityId);
        }

        /**
         * @brief clones a component from a source to a destination entity
         */
        void clone_component(id_type dst, id_type src) override
        {
            OPTICK_EVENT();
            static_assert(std::is_copy_constructible<component_type>::value,
                "cannot copy component, therefore component cannot be cloned onto new entity!");

            {
                async::readwrite_guard guard(m_lock);
                m_components[dst] = m_components[src];
            }

            m_eventBus->raiseEvent<events::component_creation<component_type>>(entity_handle(dst));
        }
    };
}
