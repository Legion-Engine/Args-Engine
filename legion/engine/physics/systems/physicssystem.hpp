#pragma once
#include <core/core.hpp>
#include <physics/components/rigidbody.hpp>
#include <physics/data/physics_manifold_precursor.hpp>
#include <physics/data/physics_manifold.hpp>
#include <physics/physics_contact.hpp>
#include <physics/components/physics_component.hpp>
#include <physics/data/identifier.hpp>
#include <physics/events/events.hpp>
#include <memory>
#include <rendering/debugrendering.hpp>
#include <physics/components/fracturer.hpp>

namespace legion::physics
{
    struct MeshLine
    {
        math::vec3 start;
        math::vec3 end;
        math::color Color;
    };

    typedef std::shared_ptr<PhysicsCollider> PhysicsColliderPtr;

    class PhysicsSystem final : public System<PhysicsSystem>
    {
    public:
        static bool IsPaused;
        static bool oneTimeRunActive;

        ecs::EntityQuery rigidbodyIntegrationQuery;

        //TODO move implementation to a seperate cpp file

        virtual void setup()
        {
            createProcess<&PhysicsSystem::fixedUpdate>("Physics", m_timeStep);

            rigidbodyIntegrationQuery = createQuery<rigidbody, position, rotation,physicsComponent>();

            auto broadPhaseLambda = [this]
            (std::vector<physics_manifold_precursor>& manifoldPrecursors
                , std::vector<std::vector<physics_manifold_precursor>>& manifoldPrecursorGrouping)
            {
                bruteForceBroadPhase(manifoldPrecursors, manifoldPrecursorGrouping);
            };

            m_optimizeBroadPhase = broadPhaseLambda;

        }

        void fixedUpdate(time::time_span<fast_time> deltaTime)
        {
            rigidbodyIntegrationQuery.queryEntities();
            if (!IsPaused)
            {
                integrateRigidbodies(deltaTime);
                runPhysicsPipeline(deltaTime);
                integrateRigidbodyQueryPositionAndRotation(deltaTime);
                
            }

            if (oneTimeRunActive)
            {
                oneTimeRunActive = false;

                integrateRigidbodies(deltaTime);
                runPhysicsPipeline(deltaTime);
                integrateRigidbodyQueryPositionAndRotation(deltaTime);
              
            }
        }

        //The following function is public static so that it can be called by testSystem

        /**@brief recursively goes through the world to retrieve the physicsComponent of entities that have one
        * @param [out] manifoldPrecursors A std::vector that will store the created physics_manifold_precursor from the scene graph iteration
        * @param initialEntity The entity where you would like to start the retrieval. If you would like to iterate through the entire scene
        * put the world as a parameter
        * @param parentTransform The world transform of the initialEntity. If 'initialEntity' is the world, parentTransform would be the identity matrix
        * @param id An integer that is used to identiy a physics_manifold_precursor from one another
        */
        inline static void recursiveRetrievePreManifoldData(std::vector<physics_manifold_precursor>& manifoldPrecursors,
            const ecs::entity_handle& initialEntity, math::mat4 parentTransform = math::mat4(1.0f), int id = 0)
        {
            math::mat4 rootTransform = parentTransform;

            auto rotationHandle = initialEntity.get_component_handle<rotation>();
            auto positionHandle = initialEntity.get_component_handle<position>();
            auto scaleHandle = initialEntity.get_component_handle<scale>();
            auto physicsComponentHandle = initialEntity.get_component_handle<physicsComponent>();

            bool hasTransform = rotationHandle && positionHandle && scaleHandle;
            bool hasNecessaryComponentsForPhysicsManifold = hasTransform && physicsComponentHandle;

            int colliderID = id;
        
             //if the entity has a physicsComponent and a transform
            if (hasNecessaryComponentsForPhysicsManifold)
            {
                rotation rot = rotationHandle.read();
                position pos = positionHandle.read();
                scale scale = scaleHandle.read();

                //assemble the local transform matrix of the entity
                math::mat4 localTransform;
                math::compose(localTransform, scale, rot, pos);

                //multiply it with the parent to get the world transform
                rootTransform = parentTransform * localTransform;

                //create its physics_manifold_precursor
                physics_manifold_precursor manifoldPrecursor(rootTransform, physicsComponentHandle, colliderID);

                auto physicsComponent = physicsComponentHandle.read();

                for (auto physCollider : *physicsComponent.colliders)
                {
                    //physCollider->DrawColliderRepresentation(rootTransform);
                    physCollider->UpdateTransformedTightBoundingVolume(rootTransform);
                }

                manifoldPrecursors.push_back(manifoldPrecursor);
            }

            //call recursiveRetrievePreManifoldData on its children
            for (int i = 0; i < initialEntity.child_count(); i++)
            {
                colliderID++;
                auto child = initialEntity.get_child(i);
                recursiveRetrievePreManifoldData(manifoldPrecursors, child, rootTransform, colliderID);
            }


        }


    private:

        legion::delegate<void(std::vector<physics_manifold_precursor>&, std::vector<std::vector<physics_manifold_precursor>>&)> m_optimizeBroadPhase;
        const float m_timeStep = 0.02f;

        /** @brief Performs the entire physics pipeline (
         * Broadphase Collision Detection, Narrowphase Collision Detection, and the Collision Resolution)
        */
        void runPhysicsPipeline(float dt)
        {
            //-------------------------------------------------Broadphase Optimization-----------------------------------------------//
            int initialColliderID = 0;

            //recursively get all physics components from the world
            std::vector<physics_manifold_precursor> manifoldPrecursors;
            recursiveRetrievePreManifoldData(manifoldPrecursors, ecs::entity_handle(world_entity_id), math::mat4(1.0f), initialColliderID);

            //log::debug(" manifold precursor {}", manifoldPrecursors.size());

            std::vector<std::vector<physics_manifold_precursor>> manifoldPrecursorGrouping;
            m_optimizeBroadPhase(manifoldPrecursors, manifoldPrecursorGrouping);

            //------------------------------------------------------ Narrowphase -----------------------------------------------------//
            std::vector<physics_manifold> manifoldsToSolve;

            for (auto& manifoldPrecursor : manifoldPrecursorGrouping)
            {
                if (manifoldPrecursor.size() == 0) { continue; }

                for (int i = 0; i < manifoldPrecursor.size()-1; i++)
                {
                    for (int j = i+1; j < manifoldPrecursor.size(); j++)
                    {
                        assert(j != manifoldPrecursor.size());

                        physics_manifold_precursor& precursorA = manifoldPrecursor.at(i);
                        physics_manifold_precursor& precursorB = manifoldPrecursor.at(j);

                        auto phyCompHandleA = precursorA.physicsComponentHandle;
                        auto phyCompHandleB = precursorB.physicsComponentHandle;

                        physicsComponent precursorPhyCompA = phyCompHandleA.read();
                        physicsComponent precursorPhyCompB = phyCompHandleB.read();

                        auto precursorRigidbodyA = phyCompHandleA.entity.get_component_handle<rigidbody>();
                        auto precursorRigidbodyB = phyCompHandleB.entity.get_component_handle<rigidbody>();

                        //only construct a manifold if at least one of these requirement are fulfilled
                        //1. One of the physicsComponents is a trigger and the other one is not
                        //2. One of the physicsComponent's entity has a rigidbody and the other one is not a trigger
                        //3. Both have a rigidbody

                        bool isBetweenTriggerAndNonTrigger =
                            (precursorPhyCompA.isTrigger && !precursorPhyCompB.isTrigger) || (!precursorPhyCompA.isTrigger && precursorPhyCompB.isTrigger);

                        bool isBetweenRigidbodyAndNonTrigger =
                            (precursorRigidbodyA && !precursorPhyCompB.isTrigger) || (precursorRigidbodyB && !precursorPhyCompA.isTrigger);

                        bool isBetween2Rigidbodies = (precursorRigidbodyA && precursorRigidbodyB);


                        if (isBetweenTriggerAndNonTrigger || isBetweenRigidbodyAndNonTrigger || isBetween2Rigidbodies)
                        {
                            constructManifoldsWithPrecursors(manifoldPrecursor.at(i), manifoldPrecursor.at(j),
                                manifoldsToSolve,
                                precursorRigidbodyA || precursorRigidbodyB
                                , precursorPhyCompA.isTrigger || precursorPhyCompB.isTrigger);
                        }
                    }
                }
            }

            //------------------------------------------------ Pre Colliison Solve Events --------------------------------------------//

            // all manifolds are initially valid

            std::vector<bool> manifoldValidity(manifoldsToSolve.size());

            std::fill(manifoldValidity.begin(),manifoldValidity.end(),true );

            //TODO we are currently hard coding fracture, this should be an event at some point

            for (size_t i = 0; i < manifoldsToSolve.size(); i++)
            {
                auto& manifold = manifoldsToSolve.at(i);

                auto entityHandleA = manifold.physicsCompA.entity;
                auto entityHandleB = manifold.physicsCompB.entity;

                auto fracturerHandleA = entityHandleA.get_component_handle<Fracturer>();
                auto fracturerHandleB = entityHandleB.get_component_handle<Fracturer>();

                bool currentManifoldValidity = manifoldValidity.at(i);

                //log::debug("- A Fracture Check");

                if (fracturerHandleA)
                {
                    auto fracturerA = fracturerHandleA.read();
                    //log::debug(" A is fracturable");
                    fracturerA.HandleFracture(manifold, currentManifoldValidity, true);

                    fracturerHandleA.write(fracturerA);
                }

                //log::debug("- B Fracture Check");

                if (fracturerHandleB)
                {
                    auto fracturerB = fracturerHandleB.read();
                    //log::debug(" B is fracturable");
                    fracturerB.HandleFracture(manifold, currentManifoldValidity,false);

                    fracturerHandleB.write(fracturerB);
                }

                manifoldValidity.at(i) = currentManifoldValidity;
            }

            //-------------------------------------------------- Collision Solver ---------------------------------------------------//
            //for both contact and friction resolution, an iterative algorithm is used.
            //Everytime physics_contact::resolveContactConstraint is called, the rigidbodies in question get closer to the actual
            //"correct" linear and angular velocity (Projected Gauss Seidel). For the sake of simplicity, an arbitrary number is set for the
            //iteration count.

            //the effective mass remains the same for every iteration of the solver. This means that we can precalculate it before
            //we start the solver

            //log::debug("--------------Logging contacts for manifold -------------------");

            initializeManifolds(manifoldsToSolve, manifoldValidity);

            //resolve contact constraint
            for (size_t contactIter = 0;
                contactIter < constants::contactSolverIterationCount; contactIter++)
            {
                resolveContactConstraint(manifoldsToSolve, manifoldValidity, dt, contactIter);
            }
            
            //resolve friction constraint
            for (size_t frictionIter = 0;
                frictionIter < constants::frictionSolverIterationCount; frictionIter++)
            {
                resolveFrictionConstraint(manifoldsToSolve, manifoldValidity);
            }

           
            //reset convergance identifiers for all colliders
            for (auto& manifold : manifoldsToSolve)
            {
                manifold.colliderA->converganceIdentifiers.clear();
                manifold.colliderB->converganceIdentifiers.clear();
            }

            //using the known lambdas of this time step, add it as a convergance identifier
            for (auto& manifold : manifoldsToSolve)
            {
                for (auto& contact : manifold.contacts)
                {
                    contact.refCollider->AddConverganceIdentifier(contact);
                }
            }




        }



        /**@brief moves all physics_manifold_precursor into a singular std::vector. This esentially means that no optimization was done.
        * @note This should only be used for testing/debugging purposes
        */
        void bruteForceBroadPhase(std::vector<physics_manifold_precursor>& manifoldPrecursors,
            std::vector<std::vector<physics_manifold_precursor>>& manifoldPrecursorGrouping);

        


        /**@brief given 2 physics_manifold_precursors precursorA and precursorB, create a manifold for each collider in precursorA
        * with every other collider in precursorB. The manifolds that involve rigidbodies are then pushed into the given manifold list
        * @param manifoldsToSolve [out] a std::vector of physics_manifold that will store the manifolds created
        * @param isRigidbodyInvolved A bool that indicates whether a rigidbody is involved in this manifold
        * @param isTriggerInvolved A bool that indicates whether a physicsComponent with a physicsComponent::isTrigger set to true is involved in this manifold
        */
        void constructManifoldsWithPrecursors(physics_manifold_precursor& precursorA, physics_manifold_precursor& precursorB,
            std::vector<physics_manifold>& manifoldsToSolve, bool isRigidbodyInvolved, bool isTriggerInvolved)
        {
            auto physicsComponentA = precursorA.physicsComponentHandle.read();
            auto physicsComponentB = precursorB.physicsComponentHandle.read();

            for (auto colliderA : *physicsComponentA.colliders)
            {
                for (auto colliderB : *physicsComponentB.colliders)
                {
                    physics::physics_manifold m;
                    constructManifoldWithCollider(colliderA, colliderB, precursorA, precursorB, m);

                    if (!m.isColliding)
                    {
                        continue;
                    }

                    colliderA->PopulateContactPoints(colliderB, m);

                    if (isRigidbodyInvolved && !isTriggerInvolved)
                    {
                        manifoldsToSolve.push_back(m);
                        raiseEvent<collision_event>(m,m_timeStep);
                    }

                    if (isTriggerInvolved)
                    {
                        //notify the event-bus
                        raiseEvent<trigger_event>(m,m_timeStep);
                        //notify both the trigger and triggerer
                        //TODO:(Developer-The-Great): the triggerer and trigger should probably received this event
                        //TODO:(cont.) through the event bus, we should probably create a filterable system here to
                        //TODO:(cont.) uniquely identify involved objects and then redirect only required messages
                    }

                }
            }
        }

        void constructManifoldWithCollider(
            PhysicsColliderPtr colliderA, PhysicsColliderPtr colliderB
            , physics_manifold_precursor& precursorA, physics_manifold_precursor& precursorB, physics_manifold& manifold)
        {
            manifold.colliderA = colliderA;
            manifold.colliderB = colliderB;

            manifold.physicsCompA = precursorA.physicsComponentHandle;
            manifold.physicsCompB = precursorB.physicsComponentHandle;

            manifold.transformA = precursorA.worldTransform;
            manifold.transformB = precursorB.worldTransform;

            // log::debug("colliderA->CheckCollision(colliderB, manifold)");
            colliderA->CheckCollision(colliderB, manifold);

        }

        /** @brief gets all the entities with a rigidbody component and calls the integrate function on them
        */
        void integrateRigidbodies(float deltaTime)
        {
            rigidbodyIntegrationQuery.queryEntities();
            for (auto ent : rigidbodyIntegrationQuery)
            {
                auto rbPosHandle = ent.get_component_handle<position>();
                auto rbRotHandle = ent.get_component_handle<rotation>();
                auto rbRigidbodyHandle = ent.get_component_handle<rigidbody>();

                integrateRigidbody(rbPosHandle, rbRotHandle, rbRigidbodyHandle, deltaTime);
            }
        }

        /** @brief given a set of component handles, updates the position and orientation of an entity with a rigidbody component.
        */
        void integrateRigidbody(ecs::component_handle<position>& posHandle
            , ecs::component_handle<rotation>& rotHandle, ecs::component_handle<rigidbody>& rbHandle, float dt)
        {
            auto rb = rbHandle.read();
            auto rbPos = posHandle.read();
            auto rbRot = rotHandle.read();

            ////-------------------- update position ------------------//
            math::vec3 acc = rb.forceAccumulator * rb.inverseMass;
            rb.velocity += (acc + constants::gravity) * dt;

            ////-------------------- update rotation ------------------//
            math::vec3 angularAcc = rb.torqueAccumulator * rb.globalInverseInertiaTensor;
            rb.angularVelocity += (angularAcc)* dt;

            rb.resetAccumulators();

            rbHandle.write(rb);
            posHandle.write(rbPos);
            rotHandle.write(rbRot);

        }

        void integrateRigidbodyQueryPositionAndRotation(float deltaTime)
        {
            for (auto ent : rigidbodyIntegrationQuery)
            {
                auto rbPosHandle = ent.get_component_handle<position>();
                auto rbRotHandle = ent.get_component_handle<rotation>();
                auto rbRigidbodyHandle = ent.get_component_handle<rigidbody>();
                auto rbPhysicsComponentHandle = ent.get_component_handle<physicsComponent>();

                integrateRigidbodyPositionAndRotations(rbPosHandle, rbRotHandle
                    , rbRigidbodyHandle, rbPhysicsComponentHandle, deltaTime);

            }
        }

        void integrateRigidbodyPositionAndRotations(ecs::component_handle<position>& posHandle
            , ecs::component_handle<rotation>& rotHandle, ecs::component_handle<rigidbody>& rbHandle,
            ecs::component_handle<physicsComponent>& physicsComponentHandle, float dt)
        {
            auto rb = rbHandle.read();
            auto rbPos = posHandle.read();
            auto rbRot = rotHandle.read();

            ////-------------------- update position ------------------//
            rbPos += rb.velocity * dt;

            ////-------------------- update rotation ------------------//
            float angle = math::clamp(math::length(rb.angularVelocity), 0.0f, 32.0f);
            float dtAngle = angle * dt;

            math::quat bfr = rbRot;

            if (!math::epsilonEqual(dtAngle, 0.0f, math::epsilon<float>()))
            { 
                math::vec3 axis = math::normalize(rb.angularVelocity);

                math::quat glmQuat = math::angleAxis(dtAngle, axis);
                rbRot = glmQuat * rbRot;
                rbRot = math::normalize(rbRot);
            }

            math::quat afr = rbRot;



            //for now assume that there is no offset from bodyP
            rb.globalCentreOfMass = rbPos;

            rb.UpdateInertiaTensor(rbRot);

            rbHandle.write(rb);
            posHandle.write(rbPos);
            rotHandle.write(rbRot);

        }

        void initializeManifolds(std::vector<physics_manifold>& manifoldsToSolve, std::vector<bool>& manifoldValidity)
        {
            for (int i = 0; i < manifoldsToSolve.size(); i++)
            {
                if (manifoldValidity.at(i))
                {
                    auto& manifold = manifoldsToSolve.at(i);

                    for (auto& contact : manifold.contacts)
                    {
                        contact.preCalculateEffectiveMass();
                        contact.ApplyWarmStarting();
                    }
                }

            }
        }

        void resolveContactConstraint(std::vector<physics_manifold>& manifoldsToSolve, std::vector<bool>& manifoldValidity,float dt,int contactIter)
        {
            for (int manifoldIter = 0;
                manifoldIter < manifoldsToSolve.size(); manifoldIter++)
            {
                if (manifoldValidity.at(manifoldIter))
                {
                    auto& manifold = manifoldsToSolve.at(manifoldIter);

                    for (auto& contact : manifold.contacts)
                    {
                        contact.resolveContactConstraint(dt, contactIter);
                    }
                }
            }
        }

        void resolveFrictionConstraint(std::vector<physics_manifold>& manifoldsToSolve, std::vector<bool>& manifoldValidity)
        {
            for (int manifoldIter = 0;
                manifoldIter < manifoldsToSolve.size(); manifoldIter++)
            {
                if (manifoldValidity.at(manifoldIter))
                {
                    auto& manifold = manifoldsToSolve.at(manifoldIter);

                    for (auto& contact : manifold.contacts)
                    {
                        contact.resolveFrictionConstraint();
                    }
                }
            }
        }

    };
}

