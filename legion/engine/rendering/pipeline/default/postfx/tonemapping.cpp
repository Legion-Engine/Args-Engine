#include <rendering/pipeline/default/postfx/tonemapping.hpp>

namespace legion::rendering
{
    std::atomic<id_type> Tonemapping::m_currentShader = { nameHash("aces tonemapping") };

    void Tonemapping::setAlgorithm(tonemapping_type type)
    {
        OPTICK_EVENT();
        static id_type acesId = nameHash("aces tonemapping");
        static id_type reinhardId = nameHash("reinhard tonemapping");
        static id_type reinhardJodieId = nameHash("reinhard jodie tonemapping");
        static id_type legionId = nameHash("legion tonemapping");
        static id_type unreal3Id = nameHash("unreal3 tonemapping");

        switch (type)
        {
        case tonemapping_type::aces:
            m_currentShader.store(acesId, std::memory_order_relaxed);
            break;
        case tonemapping_type::reinhard:
            m_currentShader.store(reinhardId, std::memory_order_relaxed);
            break;
        case tonemapping_type::reinhard_jodie:
            m_currentShader.store(reinhardJodieId, std::memory_order_relaxed);
            break;
        case tonemapping_type::legion:
            m_currentShader.store(legionId, std::memory_order_relaxed);
            break;
        case tonemapping_type::unreal3:
            m_currentShader.store(unreal3Id, std::memory_order_relaxed);
            break;
        default:
            m_currentShader.store(legionId, std::memory_order_relaxed);
            break;
        }
    }

    void Tonemapping::setup(app::window& context)
    {
        OPTICK_EVENT();
        using namespace legion::core::fs::literals;
        rendering::ShaderCache::create_shader("aces tonemapping", "engine://shaders/aces.shs"_view);
        rendering::ShaderCache::create_shader("reinhard tonemapping", "engine://shaders/reinhard.shs"_view);
        rendering::ShaderCache::create_shader("reinhard jodie tonemapping", "engine://shaders/reinhardjodie.shs"_view);
        rendering::ShaderCache::create_shader("legion tonemapping", "engine://shaders/legiontonemap.shs"_view);
        rendering::ShaderCache::create_shader("unreal3 tonemapping", "engine://shaders/unreal3.shs"_view);
        addRenderPass<&Tonemapping::renderPass>();
        exposure = 0.5f;
    }

    void Tonemapping::renderPass(framebuffer& fbo, texture_handle colortexture, texture_handle depthtexture, time::span deltaTime)
    {
        OPTICK_EVENT();
        static id_type exposureId = nameHash("exposure");
        static bool firstFrame = true;

        auto tex = std::get<texture_handle>(fbo.getAttachment(GL_COLOR_ATTACHMENT0)).get_texture();

        auto size = tex.size();

        size_type maxMip = math::floor(math::log2(math::max(size.x, size.y)));

        static std::vector<math::color> colors = { math::color() };

        if (!firstFrame)
        {
            OPTICK_EVENT("Read mip pixel data");
            glBindTexture(static_cast<GLenum>(tex.type), tex.textureId);
            glGetTexImage(static_cast<GLenum>(tex.type), maxMip, components_to_format[static_cast<int>(tex.channels)], GL_FLOAT, colors.data());
            glBindTexture(static_cast<GLenum>(tex.type), 0);
        }
        else
            firstFrame = false;

        float luminance = math::dot(math::vec3(colors[0].r, colors[0].g, colors[0].b), math::vec3(0.2126f, 0.7152f, 0.0722f));

        float newExposure = math::clamp(math::pow(math::max((1.0f - luminance), 0.f), 2.2f) * 10.f, 0.f, 10.f);

        if (newExposure < exposure)
            exposure = math::lerp(exposure, newExposure, deltaTime.seconds());
        else
            exposure = math::lerp(exposure, newExposure, deltaTime.seconds() * 0.5f);

        auto shader = ShaderCache::get_handle(m_currentShader.load(std::memory_order_relaxed));

        fbo.bind();
        shader.bind();

        shader.get_uniform<float>(exposureId).set_value(exposure);

        shader.get_uniform_with_location<texture_handle>(SV_SCENECOLOR).set_value(colortexture);
        renderQuad();
        shader.release();
        fbo.release();

        {
            OPTICK_EVENT("Generate scene color mipmaps");
            glGenerateTextureMipmap(tex.textureId);
        }
    }

}
