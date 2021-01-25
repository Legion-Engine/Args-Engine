#pragma once
#include <rendering/data/particle_system_base.hpp>
#include <rendering/debugrendering.hpp>
#include <core/core.hpp>
#include <rendering/data/Octree.hpp>
#include <rendering/components/lod.hpp>
#include <random>
#include<rendering/components/point_emitter_data.hpp>
#include <rendering/components/point_cloud_particle_container.hpp>
using namespace legion;
/**
 * @struct pointCloudParameters
 * @brief A struct that simplifies the parameter input of the particle system constructor.
 */
struct pointCloudParameters
{
    math::vec3 startingSize;
    rendering::material_handle particleMaterial;
    rendering::model_handle particleModel;

    bool looping = false;
    float maxLifeTime = 0.0f;
    float startingLifeTime = 0.0f;
    uint spawnRate = 0;
    uint particleCount = 0;
    uint maxParticles = 0;
    math::vec3 startingVel = math::vec3(0);
    float sizeOverLifeTime = 0.0f;
};

class PointCloudParticleSystem : public rendering::ParticleSystemBase
{
public:
    /**
     * @brief Constructor of the point cloud particle system.
     * @param params A struct with a bunch of default parameters and some parameters needed to be set.
     * @param positions A list of positions that the particle system uses to create its particles at.
     */
    PointCloudParticleSystem(pointCloudParameters params)
    {
        m_looping = params.looping;
        m_maxLifeTime = params.maxLifeTime;
        m_startingLifeTime = params.startingLifeTime;
        m_spawnRate = params.spawnRate;
        m_particleCount = params.particleCount;
        m_maxParticles = params.maxParticles;
        m_startingVelocity = params.startingVel;
        m_startingSize = params.startingSize;
        m_sizeOverLifetime = params.sizeOverLifeTime;
        m_particleMaterial = params.particleMaterial;
        m_particleModel = params.particleModel;
        container = rendering::point_cloud_particle_container();
    }

    /**
     * @brief Setup function that will be called to populate the emitter with the required particles.
     * @param emitter_handle The emitter that you are populating.
     */
    void setup(ecs::component_handle<rendering::particle_emitter> emitter_handle) const override
    {
        auto emitter = emitter_handle.read();
        m_positions = emitter.pointInput;
        m_colors = emitter.colorInput;

        //Create data component
        auto emitterDataHandle = emitter_handle.entity.add_component<rendering::point_emitter_data>();
        auto emitterData = emitterDataHandle.read();

        //define octree bounds
        float minX = std::numeric_limits<float>().max();
        float maxX = std::numeric_limits<float>().min();

        float minY = std::numeric_limits<float>().max();
        float maxY = std::numeric_limits<float>().min();

        float minZ = std::numeric_limits<float>().max();
        float maxZ = std::numeric_limits<float>().min();
        //shuffle input
        /*auto rng = std::default_random_engine{};
        std::shuffle(m_positions.begin(), m_positions.end(), rng);*/
        for (auto position : m_positions)
        {
            if (position.x < minX) minX = position.x;
            if (position.x > maxX) maxX = position.x;
            if (position.y < minY) minY = position.y;
            if (position.y > maxY) maxY = position.y;
            if (position.z < minZ) minZ = position.z;
            if (position.z > maxZ) maxZ = position.z;
        }
        float universalMin = math::min(minX, math::min(minY, minX));
        float universalMax = math::max(maxX, math::max(maxY, maxX));

        math::vec3 min = math::vec3(universalMin);
        math::vec3 max = math::vec3(universalMax);
        //Create Octree
        emitterData.Tree = new rendering::Octree<math::color>(8, min, max);
        //insert points into tree
        int index = 0;
        for (auto position : m_positions)
        {
            emitterData.Tree->insertNode(math::color(m_colors.at(index)), position);
            index++;
        }
        //Write to handle
        emitter.container = &container;
        emitter_handle.write(emitter);
        emitterDataHandle.write(emitterData);
        //create the particles
        populateEmitter(emitter_handle, emitterDataHandle);
    }

    /**
     * @brief Creates particles based on position for the emitter, checks for recycling
     */
    void CreateParticles(std::vector<std::pair<math::vec3, math::color>>* inputData, rendering::particle_emitter& emitter, rendering::point_emitter_data& data) const
    {
        OPTICK_EVENT();

        int index = 0;
        for (auto [newPos, newColor] : *inputData)
        {
            ecs::component_handle<transform> trans = checkContainerToRecycle();

            //auto ent = trans.entity;

            //Gets position, rotation and scale of entity.
            auto newTrans = trans.entity.get_component_handles<transform>();
            auto& [pos, _, scale] = newTrans;

            //Sets the particle scale to the right scale.
        //    transRead.position
            pos.write(newPos);
            scale.write(math::vec3(m_startingSize));

            //auto it = container.colorBufferData.begin() + data.bufferPosition + data.emitterSize + index;
            container.colorBufferData.push_back(newColor);
            //Populates the particle with the appropriate stuffs.
            createParticle(trans);
            index++;

        }

        data.emitterSize += inputData->size();
    }

    /**
     * @brief Decreases the particles detail down to the specified target LOD
     */
    void decreaseDetail(
        rendering::particle_emitter& emitter,
        rendering::point_emitter_data& data,
        int targetLod,
        int maxLod,
        ecs::component_handle<rendering::point_emitter_data>& dataHandle,
        ecs::EntityQuery& enities
    ) const

    {
        OPTICK_EVENT();

        if (container.livingParticles.size() == 0) return;
        //get the amount of particles to remove
        int targetParticleCount = data.ElementsPerLOD.at(maxLod - targetLod);
        int delta = data.emitterSize - targetParticleCount;
        auto dataToRemove = data.posRangeMap.back();
        int bufferPosition = data.bufferPosition + data.emitterSize;
        bufferPosition = dataToRemove.first;
        //remove particles from the end of the living particles
        //living particles should be stored in order of detail
        for (size_t i = 0; i < dataToRemove.second; i++)
        {
            //remove first object at position for the stored size
            container.colorBufferData.erase(container.colorBufferData.begin() + dataToRemove.first);
            auto particle = container.livingParticles.at(dataToRemove.first);
            //remove renderer and push to dead particles
            particle.remove_component<rendering::mesh_renderer>();
            container.deadParticles.emplace_back(particle);
            //erase particle
            container.livingParticles.erase(container.livingParticles.begin() + dataToRemove.first);
        }
        data.emitterSize -= delta;
        data.CurrentLOD = targetLod;
        //pop back of point map
        data.posRangeMap.pop_back();

        //update other emitterbuffer positions
        std::vector<math::vec4> colorData;
        int index = 0;
        for (auto pointEntities : enities)
        {
            auto currentHandle = pointEntities.get_component_handle<rendering::point_emitter_data>();
            //skip if entity is the same
            if (dataHandle == currentHandle) continue;
            auto otherData = currentHandle.read();
            bool changes = false;
            int index = 0;
            for (auto pair : otherData.posRangeMap)
            {
                //if there is data at a higher position than the data that is beeing removed it needs to be moved back by the amount of points removed
                if (pair.first > dataToRemove.first)
                {
                    log::debug("changed data!");
                    otherData.posRangeMap.at(index).first -= dataToRemove.second;
                    changes = true;
                }
                index++;
            }
            //write data if there were changes
            if (changes)currentHandle.write(otherData);
        }
    }
    /**
    * @brief Increases the particles up to the specified target LOD
    */
    void increaseDetail(rendering::particle_emitter& emitter, rendering::point_emitter_data& data, int targetLod, int maxLod, rendering::lod& lod, ecs::EntityQuery& enities) const
    {
        OPTICK_EVENT();


        if (!data.Tree) return;

        //create data container
        std::vector<std::pair<math::vec3, math::color>>* newData = new std::vector<std::pair<math::vec3, math::color>>();
        //populate emitter progressively for each LOD
        data.Tree->GetDataRangePair(lod.MaxLod - data.CurrentLOD, lod.MaxLod - targetLod, newData);
        int size = newData->size();

        CreateParticles(newData, emitter, data);
        data.CurrentLOD = targetLod;

        //store position and amount of particles generated
        data.posRangeMap.push_back(std::make_pair<int, int>(emitter.container->livingParticles.size() - newData->size(), newData->size()));
        //push back position for higher position buffers
        newData->clear();
    }
    /**
     * @brief populates the particle emitter with particles, creates LOD component and an Octree
     */
    void populateEmitter(ecs::component_handle<rendering::particle_emitter> emitter_handle, ecs::component_handle<rendering::point_emitter_data> data) const
    {
        //read particle emitter if tree is null something went wrong, return
        rendering::particle_emitter emitter = emitter_handle.read();
        auto emitterData = data.read();
        if (!emitterData.Tree) return;
        int maxTreeDepth = emitterData.Tree->GetTreeDepth();
        //set buffer position to the size of all existing particles
        emitterData.bufferPosition = container.livingParticles.size();

        //create data container
        std::vector<std::pair<math::vec3, math::color>>* newData = new std::vector<std::pair<math::vec3, math::color>>();
        //populate emitter progressively for each LOD
        int particleCount = 0;
        int LODcount = 0;
        for (size_t i = 0; i < maxTreeDepth; i++)
        {
            emitterData.Tree->GetDataRangePair(i, (i + 1), newData);
            particleCount += newData->size();
            //exit loop if there is no new data to be found
            if (newData->size() == 0) break;
            CreateParticles(newData, emitter, emitterData);
            LODcount++;
            //store the amount of particles for the lod so that we can later easily remove them again
            emitterData.ElementsPerLOD.push_back(particleCount);
            //store the position and amount of particles
            emitterData.posRangeMap.push_back(std::make_pair<int, int>(container.livingParticles.size() - newData->size(), newData->size()));

            newData->clear();

        }
        rendering::lod lodComponent = rendering::lod(LODcount);
        emitter_handle.entity.add_component<rendering::lod>(lodComponent);
        emitterData.CurrentLOD = 0;
        //size is new size substracted by the original position
        //emitterData.emitterSize = container.livingParticles.size() - emitterData.bufferPosition;
        //decreaseDetail(emitter, emitterData, 5, maxTreeDepth);
        data.write(emitterData);
        emitter_handle.write(emitter);
    }
    void animate(rendering::particle_emitter& emitter, rendering::point_emitter_data& data)
    {

    }
    void SetColor(rendering::particle_emitter& emitter, rendering::point_emitter_data& data) const
    {
        //assign colors
        for (auto item : data.posRangeMap)
        {
            std::fill(container.colorBufferData.begin() + item.first, container.colorBufferData.begin() + item.first + item.second, math::colors::red);
        }

    }
    /**
     * @brief Checks if there has been LOD changes, decreases or increases LOD
     */
    void update(std::vector<ecs::entity_handle>& entity, ecs::component_handle<rendering::particle_emitter> emitterHandle, ecs::EntityQuery& entities, time::span) const override
    {
        OPTICK_EVENT();
        auto lodComponent = emitterHandle.entity.get_component_handle<rendering::lod>().read();
        rendering::particle_emitter emitter = emitterHandle.read();
        auto emitterDataHandle = emitterHandle.entity.get_component_handle<rendering::point_emitter_data>();
        auto emitterData = emitterDataHandle.read();
        if (emitterData.CurrentLOD != lodComponent.Level)
        {
            if (lodComponent.Level == 0)
            {
                SetColor(emitter, emitterData);
            }
            if (emitterData.CurrentLOD > lodComponent.Level)
            {
                increaseDetail(emitter, emitterData, lodComponent.Level, lodComponent.MaxLod, lodComponent, entities);
            }
            else
            {
                decreaseDetail(emitter, emitterData, lodComponent.Level, lodComponent.MaxLod, emitterDataHandle, entities);
            }
            emitterData.CurrentLOD = lodComponent.Level;
            emitterHandle.write(emitter);
            emitterDataHandle.write(emitterData);
        }
    }
    mutable rendering::point_cloud_particle_container container;


private:
    ecs::component_handle<transform> checkContainerToRecycle() const
    {
        OPTICK_EVENT();

        ecs::entity_handle particularParticle;

        if (!container.deadParticles.empty())
        {
            //Get particle from dead particle list.
            particularParticle = container.deadParticles.back();
            //remove last item
            container.deadParticles.pop_back();
        }
        else
        {
            //Create new particle entity.
            particularParticle = m_registry->createEntity();
            //give newly created particle a transform
            particularParticle.add_components<transform>();
            //  particularParticle.add_component<rendering::particle>();
        }
        //Add particle to living particle list.
        container.livingParticles.push_back(particularParticle);

        return particularParticle.get_component_handle<transform>();
    }
    mutable  std::vector<math::vec3> m_positions;
    mutable  std::vector<math::vec4> m_colors;
};
