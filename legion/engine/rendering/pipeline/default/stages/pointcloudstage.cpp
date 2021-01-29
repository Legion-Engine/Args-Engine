#include <rendering/pipeline/default/stages/pointcloudstage.hpp>

namespace legion::rendering
{
    point_cloud_particle_container* PointCloudStage::m_container;

    void PointCloudStage::SetContainer(point_cloud_particle_container* container)
    {
        m_container = container;
    }

    void PointCloudStage::buffferCloud(point_cloud_particle_container& cloud)
    {
        cloud.vertexArray = vertexarray::generate();
        cloud.colorBuffer = buffer(GL_ARRAY_BUFFER, cloud.colorBufferData, GL_DYNAMIC_DRAW);
        cloud.vertexArray.setAttribPointer(cloud.colorBuffer, SV_COLOR, 4, GL_FLOAT, false, 0, 0);

        cloud.positionBuffer = buffer(GL_ARRAY_BUFFER, cloud.positionBufferData, GL_STATIC_DRAW);
        cloud.vertexArray.setAttribPointer(cloud.positionBuffer, SV_POSITION, 3, GL_FLOAT, false, 0, 0);
        cloud.buffered = true;
    }

    void PointCloudStage::setup(app::window& context)
    {
        app::context_guard guard(context);
        m_pointShader = ShaderCache::create_shader("point cloud", fs::view("assets://shaders/point.shs"));
    }

    void PointCloudStage::render(app::window& context, camera& cam, const camera::camera_input& camInput, time::span deltaTime)
    {
        static auto pointcloudQuery = createQuery<point_cloud_particle_container>();
        static id_type mainId = nameHash("main");

        auto* fbo = getFramebuffer(mainId);
        if (!fbo)
        {
            log::error("Main frame buffer is missing.");
            abort();
            return;
        }

        app::context_guard guard(context);
        if (!guard.contextIsValid())
        {
            abort();
            return;
        }

        auto [valid, message] = fbo->verify();
        if (!valid)
        {
            log::error("Main frame buffer isn't complete: {}", message);
            abort();
            return;
        }


        fbo->bind();
        m_pointShader.bind();

        if(m_container)
        {
            auto& cloud = *m_container;
            if (!cloud.buffered)
                buffferCloud(cloud);
            else
            {
                cloud.colorBuffer.bufferData(cloud.colorBufferData);
            }


        }

        fbo->release();
    }

    priority_type PointCloudStage::priority()
    {
        return opaque_priority;
    }
}
