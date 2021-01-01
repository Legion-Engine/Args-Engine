#include <core/ecs/queryregistry.hpp>
#include <core/ecs/ecsregistry.hpp>
#include <core/ecs/entity_handle.hpp>
#include <core/ecs/entityquery.hpp>
#include <algorithm>

namespace legion::core::ecs
{
    hashed_sparse_set<QueryRegistry*> QueryRegistry::m_validRegistries;

    void QueryRegistry::addComponentType(id_type queryId, id_type componentTypeId)
    {
        OPTICK_EVENT();
        {
            async::readwrite_guard guard(m_componentLock); // In this case the lock handles both the sparse_map and the contained hashed_sparse_sets
            m_componentTypes[queryId].insert(componentTypeId); // We insert the new component type we wish to track.
        }

        // First we need to erase all the entities that no longer apply to the new query.
        std::vector<entity_handle> toRemove;

        {
            async::readonly_multiguard mguard(m_entityLock, m_componentLock);
            for (int i = 0; i < m_entityLists[queryId]->size(); i++) // Iterate over all tracked entities.
            {
                entity_handle entity = m_entityLists[queryId]->at(i); // Get the id from the keys of the map.
                if (!m_registry.getEntityData(entity).components.contains(m_componentTypes[queryId])) // Check component composition
                    toRemove.push_back(entity); // Mark for erasure if the component composition doesn't overlap with the query.
            }
        }

        if (toRemove.size() > 0)
        {
            async::readwrite_guard guard(m_entityLock);
            for (entity_handle entity : toRemove)
                m_entityLists[queryId]->erase(entity); // Erase all entities marked for erasure.
        }

        // Next we need to filter through all the entities to get all the new ones that apply to the new query.

        auto [entities, entitiesLock] = m_registry.getEntities(); // getEntities returns a pair of both the container as well as the lock that should be locked by you when operating on it.
        async::mixed_multiguard mguard(entitiesLock, async::lock_state_read, m_componentLock, async::lock_state_read, m_entityLock, async::lock_state_write); // Lock locks.

        for (entity_handle entity : entities) // Iterate over all entities.
        {
            if (m_entityLists[queryId]->contains(entity)) // If the entity is already tracked, continue to the next entity.
                continue;

            if (m_registry.getEntityData(entity).components.contains(m_componentTypes[queryId])) // Check if the queried components completely overlaps the components in the entity.
                m_entityLists[queryId]->insert(entity); // Insert entity into tracking list.
        }
    }

    void QueryRegistry::removeComponentType(id_type queryId, id_type componentTypeId)
    {
        OPTICK_EVENT();
        {
            async::readwrite_guard guard(m_componentLock);
            m_componentTypes[queryId].erase(componentTypeId); // Remove component from query list.
        }

        // Then we remove all the entities that no longer overlap with the query.
        std::vector<entity_handle> toRemove;

        {
            async::readonly_multiguard mguard(m_entityLock, m_componentLock);
            for (int i = 0; i < m_entityLists[queryId]->size(); i++) // Iterate over all tracked entities.
            {
                entity_handle entity = m_entityLists[queryId]->at(i); // Get the id from the keys of the map.
                if (!m_registry.getEntity(entity).component_composition().contains(m_componentTypes[queryId])) // Check component composition
                    toRemove.push_back(entity); // Mark for erasure if the component composition doesn't overlap with the query.
            }
        }

        {
            async::readwrite_guard guard(m_entityLock);
            for (entity_handle entity : toRemove)
                m_entityLists[queryId]->erase(entity); // Erase all entities marked for erasure.
        }
    }

    void QueryRegistry::evaluateEntityChange(id_type entityId, id_type componentTypeId, bool removal)
    {
        OPTICK_EVENT();
        entity_handle entity(entityId);

        async::mixed_multiguard mmguard(m_entityLock, async::lock_state_write, m_componentLock, async::lock_state_read); // We lock now so that we don't need to reacquire the locks every iteration.

        for (int i = 0; i < m_entityLists.size(); i++)
        {
            id_type queryId = m_entityLists.keys()[i];
            if (!m_componentTypes[queryId].contains(componentTypeId)) // This query doesn't care about this component type.
                continue;

            if (m_entityLists[queryId]->contains(entity))
            {
                if (removal)
                {
                    m_entityLists[queryId]->erase(entity); // Erase the entity from the query's tracking list if the component was removed from the entity.
                    continue;
                }
            }
            else if (m_registry.getEntityData(entityId).components.contains(m_componentTypes[queryId]))
            {
                m_entityLists[queryId]->insert(entity); // If the entity also contains all the other required components for this query, then add this entity to the tracking list.
            }
        }
    }

    void QueryRegistry::markEntityDestruction(id_type entityId)
    {
        OPTICK_EVENT();
        entity_handle entity(entityId);

        async::readwrite_guard guard(m_entityLock);
        for (int i = 0; i < m_entityLists.size(); i++) // Iterate over all query tracking lists.
        {
            id_type queryId = m_entityLists.keys()[i];
            if (m_entityLists[queryId]->contains(entity))
                m_entityLists[queryId]->erase(entity); // Erase entity from tracking list if it's present.
        }
    }

    id_type QueryRegistry::getQueryId(const hashed_sparse_set<id_type>& componentTypes)
    {
        OPTICK_EVENT();
        async::readonly_guard guard(m_componentLock);

        for (auto [id, types] : m_componentTypes)
        {
            if (types == componentTypes) // Iterate over all component type lists of all queries and check if it's the same as the requested list.
                return id;
        }

        return invalid_id;
    }

    EntityQuery QueryRegistry::createQuery(const hashed_sparse_set<id_type>& componentTypes)
    {
        OPTICK_EVENT();
        id_type queryId = getQueryId(componentTypes); // Check if a query already exists with the requested component types. 

        if (!queryId)
        {
            queryId = addQuery(componentTypes); // Create a new query if one doesn't exist yet.
        }

        return EntityQuery(queryId, this, &m_registry);
    }

    const hashed_sparse_set<id_type>& QueryRegistry::getComponentTypes(id_type queryId)
    {
        OPTICK_EVENT();
        async::readonly_guard guard(m_componentLock);
        return m_componentTypes[queryId];
    }

    id_type QueryRegistry::addQuery(const hashed_sparse_set<id_type>& componentTypes)
    {
        OPTICK_EVENT();
        id_type queryId;

        { // Write permitted critical section for m_entityLists
            async::readwrite_multiguard mguard(m_referenceLock, m_entityLock, m_componentLock);

            queryId = m_lastQueryId++;
            m_entityLists.emplace(queryId, new entity_set()); // Create a new entity tracking list.

            m_references.emplace(queryId); // Create a new reference count.

            m_componentTypes.emplace(queryId, componentTypes); // Insert component type list for query.
        }

        { // Next we need to filter through all the entities to get all the new ones that apply to the new query.
            auto [entities, entitiesLock] = m_registry.getEntities(); // getEntities returns a pair of both the container as well as the lock that should be locked by you when operating on it.
            async::mixed_multiguard mguard(entitiesLock, async::lock_state_read, m_entityLock, async::lock_state_write); // Lock locks.

            for (entity_handle entity : entities) // Iterate over all entities.
                if (m_registry.getEntityData(entity).components.contains(componentTypes)) // Check if the queried components completely overlaps the components in the entity.
                    m_entityLists[queryId]->insert(entity); // Insert entity into tracking list.
        }

        return queryId;
    }

    entity_set QueryRegistry::getEntities(id_type queryId) const
    {
        OPTICK_EVENT();
        async::readonly_guard entguard(m_entityLock);

        auto& ret = m_entityLists.get(queryId);
        OPTICK_EVENT("Copy entity set");
        return *ret;
    }

    void QueryRegistry::addReference(id_type queryId)
    {
        OPTICK_EVENT();
        async::readonly_guard refguard(m_referenceLock);

        m_references.get(queryId)++;
    }

    void QueryRegistry::removeReference(id_type queryId)
    {
        OPTICK_EVENT();
        if (queryId == invalid_id)
            return;

        async::readonly_guard refguard(m_referenceLock);

        if (!m_references.contains(queryId))
            return;

        size_type& referenceCount = m_references.get(queryId);
        referenceCount--;

        if (referenceCount == 0) // If there are no more references to this query then erase the query to reduce memory footprint.
        {
            async::readwrite_multiguard mguard(m_referenceLock, m_entityLock, m_componentLock);

            m_references.erase(queryId);
            m_entityLists.erase(queryId);
            m_componentTypes.erase(queryId);
        }
    }

    size_type QueryRegistry::getReferenceCount(id_type queryId)
    {
        OPTICK_EVENT();
        async::readonly_guard refguard(m_referenceLock);
        if (m_references.contains(queryId))
            return m_references[queryId];
        return 0;
    }
}
