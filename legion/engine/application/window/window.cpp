#include <application/window/window.hpp>
#include <application/window/windowsystem.hpp>

namespace legion::application
{
    void window::enableCursor(bool enabled) const
    {
        context_guard guard(*this);
        ContextHelper::setInputMode(handle, GLFW_CURSOR, enabled ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED);
    }

    void window::setSwapInterval(uint interval)
    {
        context_guard guard(*this);
        ContextHelper::swapInterval(interval);
        m_swapInterval = interval;
    }

    void window::show() const
    {
        ContextHelper::showWindow(handle);
    }

    int window::swapInterval() const
    {
        return m_swapInterval;
    }

    bool window::isFullscreen() const
    {
        return m_isFullscreen;
    }

    math::ivec2 window::size() const
    {
        return m_size;
    }

    math::ivec2 window::framebufferSize() const
    {
        return ContextHelper::getFramebufferSize(handle);
    }

    const std::string& window::title() const
    {
        return m_title;
    }

    context_guard::context_guard(window win) : m_win(win)
	{
        OPTICK_EVENT();
		win.lock->lock();

        if (!WindowSystem::windowStillExists(win.handle))
            return;

		m_contextIsValid = ContextHelper::makeContextCurrent(win);
		if (!m_contextIsValid)
			log::warn("Context is no longer valid.");
	}

    context_guard::~context_guard()
    {
        if (!m_contextIsValid)
            return;

        ContextHelper::makeContextCurrent(nullptr);
        m_win.lock->unlock();
    }
}
