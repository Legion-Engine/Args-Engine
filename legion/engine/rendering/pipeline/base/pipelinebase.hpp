#pragma once
#include <rendering/data/framebuffer.hpp>
#include <rendering/data/buffer.hpp>
#include <rendering/components/camera.hpp>
#include <application/application.hpp>

#include <memory>
#include <any>

namespace legion::rendering
{
    class RenderPipelineBase
    {
        friend class Renderer;
    protected:
        sparse_map<id_type, framebuffer> m_framebuffers;
        sparse_map<id_type, std::any> m_metadata;

        static ecs::EcsRegistry* m_ecs;
        static schd::Scheduler* m_scheduler;
        static events::EventBus* m_eventBus;

        bool m_abort;

        static std::atomic_bool m_exiting;

    public:
        static void exit();

        void abort();

        template<typename T>
        L_NODISCARD bool has_meta(const std::string& name);

        template<typename T, typename... Args>
        T* create_meta(const std::string& name, Args&&... args);

        template<typename T>
        L_NODISCARD T* get_meta(const std::string& name);

        template<typename T>
        L_NODISCARD bool has_meta(id_type nameHash);

        template<typename T, typename... Args>
        T* create_meta(id_type nameHash, Args&&... args);

        template<typename T>
        L_NODISCARD T* get_meta(id_type nameHash);

        framebuffer* addFramebuffer(const std::string& name, GLenum target = GL_FRAMEBUFFER);
        L_NODISCARD bool hasFramebuffer(const std::string& name, GLenum target = GL_FRAMEBUFFER);
        L_NODISCARD framebuffer* getFramebuffer(const std::string& name);
        framebuffer* addFramebuffer(id_type nameHash, GLenum target = GL_FRAMEBUFFER);
        L_NODISCARD bool hasFramebuffer(id_type nameHash, GLenum target = GL_FRAMEBUFFER);
        L_NODISCARD framebuffer* getFramebuffer(id_type nameHash);

        virtual void init(app::window& context) LEGION_PURE;

        virtual void render(app::window& context, camera& cam, const camera::camera_input& camInput, time::span deltaTime) LEGION_PURE;
    };

}
