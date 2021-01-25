#pragma once
#include <core/core.hpp>

namespace legion::physics
{
    struct VoronoiAABB
    {
        VoronoiAABB(float pUp, float pDown, float pLeft,
            float pRight, float pForward, float pBackward) :
            up(pUp),down(pDown),left(pLeft),
            right(pRight),forward(pForward),backward(pBackward)
        {

        }

        float up;
        float down;
        float left;
        float right;
        float forward;
        float backward;

    };


}
