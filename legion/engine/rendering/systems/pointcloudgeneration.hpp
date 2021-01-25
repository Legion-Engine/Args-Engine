#pragma once
#include<core/core.hpp>
#include <core/math/math.hpp>
#include <core/logging/logging.hpp>

#include <core/compute/context.hpp>
#include <core/compute/kernel.hpp>
#include <core/compute/high_level/function.hpp>

#include <rendering/systems/pointcloud_particlesystem.hpp>
#include <rendering/components/point_cloud.hpp>
#include <rendering/components/particle_emitter.hpp>
#include <rendering/components/lod.hpp>
using namespace legion;


namespace legion::rendering
{

    /**@class PointCloudGeneration
     * @brief A system that iterates all queried entities containing point_cloud and generates a particle system for them.
     */
    class PointCloudGeneration : public System<PointCloudGeneration>
    {
    public:

        /**@brief Setup inits the compute shader to sample the point clouds, creates update and does one initial generation.
          */
        void setup()
        {
            InitComputeShader();

            createProcess<&PointCloudGeneration::Update>("Update");

            Generate();
        }
        /**@Brief Every Update call Generate to check if new objects need to be generated
         */
        void Update(time::span deltaTime)
        {
            Generate();
        }

    private:
        //index to index the particle system name
        int cloudGenerationCount = 0;
        //query containing point clouds
        ecs::EntityQuery query = createQuery<point_cloud>();
        //compute shader
        compute::function pointCloudGeneratorCS;
        compute::function preProcessPointCloudCS;

        ParticleSystemHandle particleSystem;
        void InitComputeShader()
        {
            if (!pointCloudGeneratorCS.isValid())
                pointCloudGeneratorCS = fs::view("assets://kernels/pointRasterizer.cl").load_as<compute::function>("Main");
            if (!preProcessPointCloudCS.isValid())
                preProcessPointCloudCS = fs::view("assets://kernels/calculatePoints.cl").load_as<compute::function>("Main");
        }
        //query entities and iterate them
        void Generate()
        {
            query.queryEntities();
            for (auto& ent : query)
            {
                GeneratePointCloud(ent.get_component_handle<point_cloud>());
            }
        }
        //generates point clouds
        void GeneratePointCloud(ecs::component_handle<point_cloud> pointCloud)
        {
            using compute::in, compute::out, compute::karg;
            auto realPointCloud = pointCloud.read();

            //exit early if point cloud has already been generated
            if (realPointCloud.m_hasBeenGenerated) return;
            //read position
            math::vec3 posiitonOffset = pointCloud.entity.get_component_handle<position>().read();
            //get mesh data
            auto m = realPointCloud.m_mesh.get();
            auto vertices = m.second.vertices;
            auto indices = m.second.indices;
            auto uvs = m.second.uvs;
            uint triangle_count = indices.size() / 3;
            //compute process size
            uint process_Size = triangle_count;

            //generate initial buffers from triangle info
            std::vector<uint> samplesPerTri(triangle_count);
            auto vertexBuffer = compute::Context::createBuffer(vertices, compute::buffer_type::READ_BUFFER, "vertices");
            auto indexBuffer = compute::Context::createBuffer(indices, compute::buffer_type::READ_BUFFER, "indices");
            uint totalSampleCount = 0;
            uint samplesPerTriangle = realPointCloud.m_maxPoints / triangle_count;




            ///PreProcess pointcloud
            //preprocess, calculate individual sample count per triangle
            std::vector<uint> output(triangle_count);
            auto outBuffer = compute::Context::createBuffer(output, compute::buffer_type::WRITE_BUFFER, "pointsCount");
            auto computeResult = preProcessPointCloudCS
            (
                process_Size,
                vertexBuffer,
                indexBuffer,
                karg(samplesPerTriangle, "samplesPerTri"),
                outBuffer
            );
            //accumulate toutal triangle sample count
            for (size_t i = 0; i < triangle_count; i++)
            {
                totalSampleCount += output.at(i);
            }
            log::debug(totalSampleCount);




            ///Generate Point cloud
            //Generate points result vector
            std::vector<math::vec4> result(totalSampleCount);
            std::vector<math::vec4> resultColor(totalSampleCount);
            //Get normal map
            auto [lock, normal] = realPointCloud.m_heightMap.get_raw_image();
            {
                auto [lock2, albedo] = realPointCloud.m_AlbedoMap.get_raw_image();
                {
                    async::readonly_multiguard guard(lock, lock2);

                    auto normalMapBuffer = compute::Context::createImage(normal, compute::buffer_type::READ_BUFFER, "normalMap");



                    //Create buffers
                    auto albedoMapBuffer = compute::Context::createImage(albedo, compute::buffer_type::READ_BUFFER, "albedoMap");
                    auto sampleBuffer = compute::Context::createBuffer(output, compute::buffer_type::READ_BUFFER, "samples");
                    auto uvBuffer = compute::Context::createBuffer(uvs, compute::buffer_type::READ_BUFFER, "uvs");

                    auto outBuffer = compute::Context::createBuffer(result, compute::buffer_type::WRITE_BUFFER, "points");
                    auto colorBuffer = compute::Context::createBuffer(resultColor, compute::buffer_type::WRITE_BUFFER, "colors");

                    uint size = realPointCloud.m_AlbedoMap.size().x;
                    auto computeResult = pointCloudGeneratorCS
                    (
                        process_Size,
                        vertexBuffer,
                        indexBuffer,
                        uvBuffer,
                        sampleBuffer,
                        albedoMapBuffer,
                        normalMapBuffer,
                        karg(realPointCloud.m_heightStrength, "normalStrength"),
                        karg(size, "textureSize"),
                        outBuffer,
                        colorBuffer
                    );
                }
            }
            //translate vec4 into vec3
            std::vector<math::vec3> particleInput(totalSampleCount);
            for (size_t i = 0; i < totalSampleCount; i++)
            {
                particleInput.at(i) = result.at(i).xyz + posiitonOffset;
            }
            //generate particle params
            pointCloudParameters params
            {
               math::vec3(realPointCloud.m_pointRadius),
               realPointCloud.m_Material,
               ModelCache::get_handle("billboard")
            };
            GenerateParticles(params, particleInput, resultColor, realPointCloud.m_trans);


            //write that pc has been generated
            realPointCloud.m_hasBeenGenerated = true;
            pointCloud.write(realPointCloud);
        }

        void GenerateParticles(pointCloudParameters params, std::vector<math::vec3> input, std::vector<math::vec4> inputColor, transform trans)
        {
            //generate particle system
            std::string name = typeName<PointCloudParticleSystem>();



            auto newPointCloud = ParticleSystemCache::createParticleSystem<PointCloudParticleSystem>(name, params);
            //create entity to store particle system
            auto newEnt = createEntity();

            //  newEnt.add_component <rendering::lod>();
            newEnt.add_components<transform>(trans.get<position>().read(), trans.get<rotation>().read(), trans.get<scale>().read());

            auto emitterHandle = newEnt.add_component<rendering::particle_emitter>();
            auto emitter = emitterHandle.read();
            emitter.particleSystemHandle = newPointCloud;
            emitter.pointInput = input;
            emitter.colorInput = inputColor;
            newEnt.get_component_handle<rendering::particle_emitter>().write(emitter);
            //newPointCloud.get()->setup(emitterHandle, input, inputColor);

            //increment index
            cloudGenerationCount++;
        }
    };
}

