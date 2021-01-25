#pragma once
#include <core/core.hpp>
#include <math.h>
namespace legion::rendering
{
    struct lod
    {
        lod(int maxLevel = 8, float maxDistance = 35.0f) : MaxLod(maxLevel), m_maxDistance(maxDistance)
        {
        }
        int MaxLod;
        int Level = 0;
        int MaxTreeLevel = 0;

        bool isInitialized = false;
        float m_maxDistance;
        std::vector<float> m_thresholdLevels;
    };
}
