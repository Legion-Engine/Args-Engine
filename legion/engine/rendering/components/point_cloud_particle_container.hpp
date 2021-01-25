#pragma once
#include<core/core.hpp>

namespace legion::rendering
{

    struct point_cloud_particle_container
    {
    public:
        std::vector<ecs::entity_handle> livingParticles;
        std::vector<ecs::entity_handle> deadParticles;
        std::vector<math::color> colorBufferData;
    };

}
