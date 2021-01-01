#include <rendering/systems/renderer.hpp>
#include <rendering/debugrendering.hpp>
#include <Optick/optick.h>

namespace legion::rendering
{
    delegate<RenderPipelineBase* (app::window&)> Renderer::m_pipelineProvider;

    void Renderer::debugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
    {
        OPTICK_EVENT();
        if (!log::impl::thread_names.count(std::this_thread::get_id()))
            log::impl::thread_names[std::this_thread::get_id()] = "OpenGL";

        cstring s;
        switch (source)
        {
        case GL_DEBUG_SOURCE_API:
            s = "OpenGL";
            break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER:
            s = "Shader compiler";
            break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:
            s = "Third party";
            break;
        case GL_DEBUG_SOURCE_APPLICATION:
            s = "Application";
            break;
        case GL_DEBUG_SOURCE_OTHER:
            s = "Other";
            break;
        default:
            s = "Unknown";
            break;
        }

        cstring t;

        switch (type)
        {
        case GL_DEBUG_TYPE_ERROR:
            t = "Error";
            break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            t = "Deprecation";
            break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            t = "Undefined behavior";
            break;
        case GL_DEBUG_TYPE_PERFORMANCE:
            t = "Performance";
            break;
        case GL_DEBUG_TYPE_PORTABILITY:
            t = "Portability";
            break;
        case GL_DEBUG_TYPE_MARKER:
            t = "Marker";
            break;
        case GL_DEBUG_TYPE_PUSH_GROUP:
            t = "Push";
            break;
        case GL_DEBUG_TYPE_POP_GROUP:
            t = "Pop";
            break;
        case GL_DEBUG_TYPE_OTHER:
            t = "Misc";
            break;
        default:
            t = "Unknown";
            break;
        }

        switch (severity)
        {
        case GL_DEBUG_SEVERITY_HIGH:
            log::error("[{}-{}] {}", s, t, message);
            break;
        case GL_DEBUG_SEVERITY_MEDIUM:
            log::warn("[{}-{}] {}", s, t, message);
            break;
        case GL_DEBUG_SEVERITY_LOW:
            log::debug("[{}-{}] {}", s, t, message);
            break;
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            log::trace("[{}-{}] {}", s, t, message);
            break;
        default:
            log::debug("[{}-{}] {}", s, t, message);
            break;
        }
    }

    bool Renderer::initContext(const app::window& window)
    {
        OPTICK_EVENT();
        if (!gladLoadGLLoader((GLADloadproc)app::ContextHelper::getProcAddress))
        {
            log::error("Failed to load OpenGL");
            return false;
        }
        else
        {
            glEnable(GL_DEBUG_OUTPUT);

            glDebugMessageCallback(&Renderer::debugCallback, nullptr);
            log::info("loaded OpenGL version: {}.{}", GLVersion.major, GLVersion.minor);
        }

        glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_GREATER);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CW);

        const unsigned char* vendor = glGetString(GL_VENDOR);
        const unsigned char* renderer = glGetString(GL_RENDERER);
        const unsigned char* version = glGetString(GL_VERSION);
        const unsigned char* glslVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);

        GLint major, minor;
        glGetIntegerv(GL_MAJOR_VERSION, &major);
        glGetIntegerv(GL_MINOR_VERSION, &minor);

        log::info("Initialized Renderer\n\tCONTEXT INFO\n\t----------------------------------\n\tGPU Vendor:\t{}\n\tGPU:\t\t{}\n\tGL Version:\t{}\n\tGLSL Version:\t{}\n\t----------------------------------\n", vendor, renderer, version, glslVersion);
        return true;
    }

    void Renderer::setThreadPriority()
    {
#ifdef LEGION_WINDOWS
        if (SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST))
        {
            log::info("Acquired realtime priority for rendering thread.");
        }
        else
        {
            DWORD dwError = GetLastError();
            /*if (dwError == ERROR_THREAD_)
                log::info("Rendering thread is already in realtime priority mode.");
            else*/
                log::warn("Rendering thread failed to enter realtime priority mode: {}", dwError);
        }
#endif // LEGION_WINDOWS
    }

    void Renderer::setup()
    {
        OPTICK_EVENT();
        RenderPipelineBase::m_ecs = m_ecs;
        RenderPipelineBase::m_scheduler = m_scheduler;
        RenderPipelineBase::m_eventBus = m_eventBus;

        RenderStageBase::m_ecs = m_ecs;
        RenderStageBase::m_scheduler = m_scheduler;
        RenderStageBase::m_eventBus = m_eventBus;

        bindToEvent<events::exit, &Renderer::onExit>();

        createProcess<&Renderer::render>("Rendering");

        m_scheduler->sendCommand(m_scheduler->getChainThreadId("Rendering"), [&](void* param)
            {
                OPTICK_EVENT("Initialization");
                (void)param;
                log::trace("Waiting on main window.");

                while (!world.has_component<app::window>())
                    std::this_thread::yield();

                app::window window = world.get_component_handle<app::window>().read();

                log::trace("Initializing context.");

                bool result = false;

                {
                    app::context_guard guard(window);
                    if (!guard.contextIsValid())
                    {
                        log::error("Failed to initialize context.");
                        m_initialized.store(false, std::memory_order_release);
                        return;
                    }
                    result = initContext(window);
                }

                if (!result)
                    log::error("Failed to initialize context.");
                else
                    setThreadPriority();

                m_initialized.store(result, std::memory_order_release);
            }, this);

        while (!m_initialized.load(std::memory_order_relaxed))
            std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    void Renderer::onExit(events::exit* event)
    {
        OPTICK_EVENT();
        m_exiting.store(true, std::memory_order_release);
    }

    void Renderer::render(time::span deltatime)
    {
        OPTICK_FRAME("Rendering");
        OPTICK_EVENT();
        if (m_pipelineProvider.isNull())
            return;

        static auto cameraQuery = createQuery<camera>();
        cameraQuery.queryEntities();
        for (auto ent : cameraQuery)
        {
            auto cam = ent.get_component_handle<camera>().read();
            app::window win = cam.targetWindow.read();
            if (!win)
                win = m_ecs->world.get_component_handle<app::window>().read();
            if (!win)
                continue;

            math::ivec2 viewportSize;
            {
                if (!app::WindowSystem::windowStillExists(win.handle))
                    continue;
                app::context_guard guard(win);
                if (!guard.contextIsValid())
                    continue;

                viewportSize = win.framebufferSize();
            }

            if (viewportSize.x == 0 || viewportSize.y == 0)
                continue;

            position camPos = ent.get_component_handle<position>().read();
            rotation camRot = ent.get_component_handle<rotation>().read();
            scale camScale = ent.get_component_handle<scale>().read();

            math::mat4 view(1.f);
            math::compose(view, camScale, camRot, camPos);
            view = math::inverse(view);

            math::mat4 projection = cam.get_projection(((float)viewportSize.x) / viewportSize.y);

            camera::camera_input cam_input_data(view, projection, camPos, camRot.forward(), cam.nearz, cam.farz, viewportSize);

            if (!m_exiting.load(std::memory_order_relaxed))
                m_pipelineProvider(win)->render(win, cam, cam_input_data, deltatime);
        }
    }

    L_NODISCARD RenderPipelineBase* Renderer::getPipeline(app::window& context)
    {
        OPTICK_EVENT();
        if (m_pipelineProvider.isNull())
            return nullptr;

        if (context == app::invalid_window)
            return nullptr;

        return m_pipelineProvider(context);
    }

    L_NODISCARD RenderPipelineBase* Renderer::getMainPipeline()
    {
        OPTICK_EVENT();
        if (m_pipelineProvider.isNull())
            return nullptr;

        auto context = world.get_component_handle<app::window>().read();
        if (context == app::invalid_window)
            return nullptr;

        return m_pipelineProvider(context);
    }

}
