#include <physics/systems/physicssystem.hpp>


namespace legion::physics
{



    bool PhysicsSystem::IsPaused = true;
    bool PhysicsSystem::oneTimeRunActive = false;


    void PhysicsSystem::bruteForceBroadPhase(std::vector<physics_manifold_precursor>& manifoldPrecursors,
        std::vector<std::vector<physics_manifold_precursor>>& manifoldPrecursorGrouping)
    {

        manifoldPrecursorGrouping.push_back(std::move(manifoldPrecursors));
    }
}
