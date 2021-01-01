#pragma once
#include <rendering/pipeline/base/pipeline.hpp>
#include <rendering/pipeline/default/defaultpipeline.hpp>

#include <unordered_map>

namespace legion::rendering
{
    class Renderer final : public System<Renderer>
    {
    private:
        static delegate<RenderPipelineBase*(app::window&)> m_pipelineProvider;
        std::atomic_bool m_initialized = false;
        std::atomic_bool m_exiting = false;

        static void debugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam);
        bool initContext(const app::window& window);

        void setThreadPriority();

    public:
        Renderer() : System<Renderer>()
        {
            setPipeline<DefaultPipeline>();
        }

        virtual void setup();

        void onDebugLine(events::event_base* event);

        void onExit(events::exit* event);

        void render(time::span deltatime);

        template<typename Pipeline, typename... Args, inherits_from<Pipeline, RenderPipeline<Pipeline>> = 0>
        static void setPipeline(Args&&... args);

        L_NODISCARD static RenderPipelineBase* getPipeline(app::window& context);
        L_NODISCARD static RenderPipelineBase* getMainPipeline();
    };
}

#include <rendering/systems/renderer.inl>
