#pragma once
#include <rendering/data/postprocessingeffect.hpp>

namespace legion::rendering
{
    /**
     * @class LensFlare
     * @brief A lens flare post processing effect.
     */
    class LensFlare : public PostProcessingEffect<LensFlare>
    {
        // Inherited via PostProcessingEffect
        virtual void setup(legion::application::window& context);
    };
}
