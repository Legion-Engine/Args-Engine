#pragma once

#include <rendering/data/Octree.hpp>
namespace legion::rendering
{
    /**@struct point emitter
     * @brief contains data for LODing the point cloud, stores currentLOD, the octree for the LOD, and ElementsPerLOD which define how many particles an LOD stores
     */
    struct point_emitter_data
    {
        int CurrentLOD = 0;
        rendering::Octree<math::color>* Tree;
        std::vector<int> ElementsPerLOD;
        //pos, size
        std::vector<std::pair<int, int>> posRangeMap;
        uint bufferPosition = 0;
        uint emitterSize = 0;
    };
}
