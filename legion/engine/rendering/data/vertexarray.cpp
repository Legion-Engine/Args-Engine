#include <rendering/data/vertexarray.hpp>

namespace legion::rendering
{
    vertexarray::vertexarray(std::nullptr_t t)
        : m_id([](app::gl_id& value) {
            if (!app::ContextHelper::initialized())
                return;

#if defined(LEGION_DEBUG)
            if (!app::ContextHelper::getCurrentContext())
            {
                log::error("No current context to delete vertexarray with.");
                return;
            }
#endif
            if(value)
                glDeleteVertexArrays(1, &value);
        }, invalid_id)
    {
#if defined(LEGION_DEBUG)
        if (!app::ContextHelper::getCurrentContext())
        {
            log::error("No current context to work with.");
            return;
        }
#endif
        glGenVertexArrays(1, &m_id);
    }

    vertexarray vertexarray::generate()
    {
        return vertexarray(nullptr);
    }

    void vertexarray::setAttribPointer(const buffer& buf, uint index, size_type size, GLenum type, bool normalized, size_type stride, size_type offset)
    {
        OPTICK_EVENT();
#if defined(LEGION_DEBUG)
        if (buf.target() != GL_ARRAY_BUFFER)
        {
            log::error("Attempted to set attribute pointer for non GL_ARRAY_BUFFER, attribute pointers only work for VBOs. VAO: {} Buffer: {}", m_id.value, buf.id());
        }

        if (!app::ContextHelper::getCurrentContext())
        {
            log::error("No current context to work with.");
            return;
        }
#endif

        glBindVertexArray(m_id);
        buf.bind();
        glEnableVertexAttribArray(index);
        glVertexAttribPointer(index, size, type, normalized, stride, reinterpret_cast<GLvoid*>(offset));
        buf.release();
        glBindVertexArray(0);
    }

    void vertexarray::setAttribDivisor(uint index, uint divisor)
    {
#if defined(LEGION_DEBUG)
        if (!app::ContextHelper::getCurrentContext())
        {
            log::error("No current context to work with.");
            return;
        }
#endif
        glBindVertexArray(m_id);
        glVertexAttribDivisor(index, divisor);
        glBindVertexArray(0);
    }

    void vertexarray::bind() const
    {
#if defined(LEGION_DEBUG)
        if (!app::ContextHelper::getCurrentContext())
        {
            log::error("No current context to work with.");
            return;
        }
#endif
        glBindVertexArray(m_id);
    }

    void vertexarray::release()
    {
#if defined(LEGION_DEBUG)
        if (!app::ContextHelper::getCurrentContext())
        {
            log::error("No current context to work with.");
            return;
        }
#endif
        glBindVertexArray(0);
    }

}
