#pragma once

#include <core/core.hpp>

namespace legion::debug
{
#if !defined drawLine

#define debug_line_event CONCAT_DEFINE(PROJECT_NAME, debug_line)

    struct debug_line_event final : public events::event_base
    {
        math::vec3 start;
        math::vec3 end;
        math::color color = math::colors::white;
        float width = 1.f;
        float time = 0;
        mutable float timeBuffer = 0;
        bool ignoreDepth = false;

        debug_line_event(math::vec3 start, math::vec3 end, math::color color = math::colors::white, float width = 1.f, float time = 0, bool ignoreDepth = false) : start(start), end(end), color(color), width(width), time(time), ignoreDepth(ignoreDepth) {}
        debug_line_event() = default;
        debug_line_event(const debug_line_event &) = default;
        debug_line_event(debug_line_event&&) = default;

        debug_line_event& operator=(const debug_line_event&) = default;
        debug_line_event& operator=(debug_line_event&&) = default;

        bool operator==(const debug_line_event& other) const
        {
            return start == other.start && end == other.end && color == other.color && width == other.width && ignoreDepth == other.ignoreDepth;
        }

        virtual bool unique() override { return false; }

        virtual id_type get_id()
        {
            static id_type id = nameHash("debug_line");
            return id;
        }
    };

#define drawLine CONCAT_DEFINE(PROJECT_NAME, drawLine)

    void drawLine (math::vec3 start, math::vec3 end, math::color color = math::colors::white, float width = 1.f, float time = 0, bool ignoreDepth = false)
    {
        Engine::eventbus->raiseEvent(std::unique_ptr<events::event_base>(new debug_line_event(start, end, color, width, time, ignoreDepth)));
    }

#endif
}

#if !defined(DOXY_EXCLUDE)
namespace std
{
    template<>
    struct hash<legion::debug::debug_line_event>
    {
        std::size_t operator()(legion::debug::debug_line_event const& line) const noexcept
        {
            std::hash<legion::core::math::vec3> vecHasher;
            std::hash<legion::core::math::color> colHasher;
            std::hash<float> fltHasher;
            std::hash<bool> boolHasher;

            size_t seed = 0;
            legion::core::math::detail::hash_combine(seed, vecHasher(line.start));
            legion::core::math::detail::hash_combine(seed, vecHasher(line.end));
            legion::core::math::detail::hash_combine(seed, colHasher(line.color));
            legion::core::math::detail::hash_combine(seed, fltHasher(line.width));
            legion::core::math::detail::hash_combine(seed, boolHasher(line.ignoreDepth));
            return seed;
        }
    };
}
#endif
