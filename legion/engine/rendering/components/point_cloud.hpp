#pragma once
#include<core/core.hpp>
#include <core/math/math.hpp>
#include <core/logging/logging.hpp>
#include <application/application.hpp>

#include <core/compute/context.hpp>
#include <core/compute/kernel.hpp>


namespace legion::rendering
{
    /**@class point_cloud
     * @brief Struct that holds data for the mandatory for the pointcloud generation
     */
    struct point_cloud
    {
    public:

        /**@brief Takes in a mesh handle, transform and a material, optionally it takes in max points and radius.
         * @param newMesh is used to sample the points for the point cloud.
         * @param newTrans is the position of the object / point cloud.
         * @param mat is the material used by the particle system that renders the points.
         * @param maxPoints is used to calculate the samples for each triangle of the mesh,
         * @param radius defines point scale.
         */
        point_cloud(mesh_handle newMesh, transform newTrans, material_handle mat, image_handle albedoMap, image_handle heightMap, uint maxPoints = 100, float radius = 0.2f)
        {
            m_mesh = newMesh;
            m_trans = newTrans;
            m_heightMap = heightMap;
            m_AlbedoMap = albedoMap;
            //m_samplesPerTriangle = calculateSamplesPerTriangle(maxPoints);
            m_pointRadius = radius;
            m_Material = mat;
            m_maxPoints = maxPoints;
        }
        point_cloud() = default;

    private:
        friend class PointCloudGeneration;
        //used to check if a new point cloud needs to be generated
        bool m_hasBeenGenerated = false;

        //point cloud parameters
        float m_heightStrength = 0.2f;
        float m_pointRadius;
        transform m_trans;
        mesh_handle m_mesh;
        uint m_samplesPerTriangle;
        uint m_sampleDepth;
        uint m_maxPoints;
        material_handle m_Material;
        image_handle m_heightMap;
        image_handle  m_AlbedoMap;
    };
}
