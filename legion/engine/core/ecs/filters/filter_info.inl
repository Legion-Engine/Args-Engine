#include <core/ecs/filters/filter_info.hpp>
#include <core/ecs/filters/filterregistry.hpp>
#pragma once

namespace legion::core::ecs
{
    template<typename... component_types>
    inline id_type filter_info<component_types...>::id()
    {
        return filter_id;
    }

    template<typename... component_types>
    inline bool filter_info<component_types...>::contains(id_type id) noexcept
    {
        return contains_direct(id);
    }

    template<typename... component_types>
    inline bool filter_info<component_types...>::contains(const std::unordered_set<id_type>& comp)
    {
        if (!comp.size())
            return false;

        if (!composition.size())
            return true;

        for (auto& typeId : composition)
            if (!comp.count(typeId))
                return false;
        return true;
    }

    template<typename... component_types>
    constexpr bool filter_info<component_types...>::contains_direct(id_type id) noexcept
    {
        for (int i = 0; i < composition.size(); i++)
            if (composition[i] == id)
                return true;
        return false;
    }
}