#pragma once
#include <rendering/data/shader.hpp>
#include <memory>

/**
 * @file material.hpp
 */

namespace legion::rendering
{
    struct material;

    /**@class material_parameter_base
     * @brief material parameter common base
     */
    struct material_parameter_base
    {
        friend struct material;
    private:
        static material_parameter_base* create_param(const std::string& name, const GLint& location, const GLenum& type);
    protected:
        std::string m_name;
        id_type m_id;
        id_type m_typeId;
        GLint m_location;

        material_parameter_base(const std::string& name, GLint location, id_type typeId) : m_name(name), m_id(nameHash(name)), m_typeId(typeId), m_location(location) {}

    public:
        /**@brief Get the type hash of the variable type of this parameter.
         */
        id_type type() { return m_typeId; }

        /**@internal
         */
        virtual void apply(shader_handle& shader) LEGION_PURE;
        /**@endinternal
        */
    };

    /**@class material_parameter
     * @brief Uniform parameter of a material
     * @tparam T Variable type of the parameter
     */
    template<typename T>
    struct material_parameter : public material_parameter_base
    {
        friend struct material;
    private:
        T m_value;

        virtual void apply(shader_handle& shader) override
        {
            shader.get_uniform<T>(m_id).set_value(m_value);
        }
    public:
        material_parameter(const std::string& name, GLint location) : material_parameter_base(name, location, typeHash<T>()) {}

        void set_value(const T& value) { m_value = value; }
        T get_value() const { return m_value; }
    };

    /**@class material
     * @brief Structure that keeps track of the parameterization of a certain shader.
     *        Multiple material instances can use the same shader but use different parameter values.
     */
    struct material
    {
        friend class MaterialCache;
        friend struct material_handle;
    private:
        shader_handle m_shader;

        void init(const shader_handle& shader)
        {
            m_shader = shader;
            for (auto& [name, location, type] : m_shader.get_uniform_info())
            {
                id_type hash = nameHash(name);
                m_parameters.emplace(hash, material_parameter_base::create_param(name, location, type));
                m_idOfLocation[location] = hash;
            }
        }

        std::string m_name;

        std::unordered_map<id_type, std::unique_ptr<material_parameter_base>> m_parameters;
        std::unordered_map<GLint, id_type> m_idOfLocation;
    public:
        /**@brief Bind the material to the rendering context and prepare for use.
         */
        void bind();

        /**@brief Release the material from the rendering context.
         */
        void release()
        {
            shader_handle::release();
        }

        /**@brief Set the value of a parameter by name.
         */
        template<typename T>
        void set_param(const std::string& name, const T& value);

        /**@brief Check if the material has a parameter by name.
         */
        template<typename T>
        L_NODISCARD bool has_param(const std::string& name);

        /**@brief Get the value of a parameter by name.
         */
        template<typename T>
        L_NODISCARD T get_param(const std::string& name);

        /**@brief Set the value of a parameter by location.
         */
        template<typename T>
        void set_param(GLint location, const T& value);

        /**@brief Check if the material has a parameter by location.
         */
        template<typename T>
        L_NODISCARD bool has_param(GLint location);

        /**@brief Get the value of a parameter by location.
         */
        template<typename T>
        L_NODISCARD T get_param(GLint location);

        /**@brief Get attribute bound to a certain name.
         */
        L_NODISCARD attribute get_attribute(const std::string& name)
        {
            return m_shader.get_attribute(nameHash(name));
        }
    };

    /**@class material_handle
     * @brief Cheap and safe to pass around handle to a certain material.
     *        Can be treated like a null-able reference, nullify by setting it to invalid_material_handle.
     * @ref material
     */
    struct material_handle
    {
        id_type id;

        /**@brief Bind the material to the rendering context and prepare for use.
         */
        void bind();

        /**@brief Release the material from the rendering context.
         */
        void release()
        {
            shader_handle::release();
        }

        std::string get_name();

        /**@brief Set the value of a parameter by name.
         */
        template<typename T>
        void set_param(const std::string& name, const T& value);

        /**@brief Check if the material has a parameter by name.
         */
        template<typename T>
        L_NODISCARD bool has_param(const std::string& name);

        /**@brief Get the value of a parameter by name.
         */
        template<typename T>
        L_NODISCARD T get_param(const std::string& name);

        /**@brief Set the value of a parameter by location.
         */
        template<typename T>
        void set_param(GLint location, const T& value);

        /**@brief Check if the material has a parameter by location.
         */
        template<typename T>
        L_NODISCARD bool has_param(GLint location);

        /**@brief Get the value of a parameter by location.
         */
        template<typename T>
        L_NODISCARD T get_param(GLint location);

        /**@brief Get attribute bound to a certain name.
         */
        attribute get_attribute(const std::string& name);

        bool operator==(const material_handle& other) const { return id == other.id; }

        template<typename Archive>
        void serialize(Archive& archive);
    };
    template<typename Archive>
    void material_handle::serialize(Archive& archive)
    {
        archive(id);
    }

    /**@brief Default invalid material handle.
     */
    constexpr material_handle invalid_material_handle{ invalid_id };

    /**@class MaterialCache
     * @brief Data cache for creating, storing and managing materials.
     */
    class MaterialCache
    {
        friend struct material_handle;
    private:
        static async::rw_spinlock m_materialLock;
        static std::unordered_map<id_type, material> m_materials;

        static material_handle m_invalid_material;

    public:
        /**@brief Create a new material with a certain name and shader.
         *        If a material already exists with that name it'll return a handle to the already existing material.
         * @return material_handle Handle to the newly created material or the already existing material. Handle may be invalid if the function failed.
         */
        static material_handle create_material(const std::string& name, const shader_handle& shader);

        /**@brief Create a new material with a certain name and shader.
         *        If a new material is created it will also load the shader if it wasn't loaded before using the shader cache.
         *        If a material already exists with that name it'll return a handle to the already existing material.
         * @return material_handle Handle to the newly created material or the already existing material. Handle may be invalid if the function failed.
         * @ref ShaderCache
         */
        static material_handle create_material(const std::string& name, const filesystem::view& shaderFile, shader_import_settings settings = default_shader_settings);

        /**@brief Get a handle to a material with a certain name.
         * @return material_handle Handle to a material attached to the given name, may be invalid if there is no material attached to that name yet.
         */
        static material_handle get_material(const std::string& name);
    };

#pragma region implementations
    template<typename T>
    void material_handle::set_param(const std::string& name, const T& value)
    {
        async::readonly_guard guard(MaterialCache::m_materialLock);
        MaterialCache::m_materials[id].set_param<T>(name, value);
    }

    template<typename T>
    void material_handle::set_param(GLint location, const T& value)
    {
        async::readonly_guard guard(MaterialCache::m_materialLock);
        MaterialCache::m_materials[id].set_param<T>(location, value);
    }

    template<typename T>
    L_NODISCARD bool material_handle::has_param(const std::string& name)
    {
        async::readonly_guard guard(MaterialCache::m_materialLock);
        return MaterialCache::m_materials[id].has_param<T>(name);
    }

    template<typename T>
    L_NODISCARD bool material_handle::has_param(GLint location)
    {
        async::readonly_guard guard(MaterialCache::m_materialLock);
        return MaterialCache::m_materials[id].has_param<T>(location);
    }

    template<typename T>
    L_NODISCARD T material_handle::get_param(const std::string& name)
    {
        async::readonly_guard guard(MaterialCache::m_materialLock);
        return MaterialCache::m_materials[id].get_param<T>(name);
    }

    template<typename T>
    L_NODISCARD T material_handle::get_param(GLint location)
    {
        async::readonly_guard guard(MaterialCache::m_materialLock);
        return MaterialCache::m_materials[id].get_param<T>(location);
    }

    template<>
    inline void material::set_param<math::color>(const std::string& name, const math::color& value)
    {
        id_type id = nameHash(name);
        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<math::vec4>())
            static_cast<material_parameter<math::vec4>*>(m_parameters[id].get())->set_value(value);
        else
            log::warn("material {} does not have a parameter named {} of type {}", m_name, name, typeName<math::color>());
    }

    template<>
    L_NODISCARD inline bool material::has_param<math::color>(const std::string& name)
    {
        id_type id = nameHash(name);
        return m_parameters.count(id) && m_parameters[id]->type() == typeHash<math::vec4>();
    }

    template<>
    L_NODISCARD inline math::color material::get_param<math::color>(const std::string& name)
    {
        id_type id = nameHash(name);
        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<math::vec4>())
            return static_cast<material_parameter<math::vec4>*>(m_parameters[id].get())->get_value();

        log::warn("material {} does not have a parameter named {} of type {}", m_name, name, typeName<math::color>());
        return math::color();
    }

    template<>
    inline void material::set_param<math::color>(GLint location, const math::color& value)
    {
        if (!m_idOfLocation.count(location))
            log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<math::color>());

        id_type id = m_idOfLocation[location];

        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<math::vec4>())
            static_cast<material_parameter<math::vec4>*>(m_parameters[id].get())->set_value(value);
        else
            log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<math::color>());
    }

    template<>
    L_NODISCARD inline math::color material::get_param<math::color>(GLint location)
    {
        if (!m_idOfLocation.count(location))
            log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<math::color>());

        id_type id = m_idOfLocation[location];
        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<math::vec4>())
            return static_cast<material_parameter<math::vec4>*>(m_parameters[id].get())->get_value();

        log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<math::color>());
        return math::color();
    }

    template<>
    L_NODISCARD inline bool material::has_param<math::color>(GLint location)
    {
        if (!m_idOfLocation.count(location))
            return false;

        id_type id = m_idOfLocation[location];
        return m_parameters.count(id) && m_parameters[id]->type() == typeHash<math::vec4>();
    }

    template<typename T>
    void material::set_param(const std::string& name, const T& value)
    {
        id_type id = nameHash(name);
        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<T>())
            static_cast<material_parameter<T>*>(m_parameters[id].get())->set_value(value);
        else
            log::warn("material {} does not have a parameter named {} of type {}", m_name, name, typeName<T>());
    }

    template<typename T>
    L_NODISCARD bool material::has_param(const std::string& name)
    {
        id_type id = nameHash(name);
        return m_parameters.count(id) && m_parameters[id]->type() == typeHash<T>();
    }

    template<typename T>
    L_NODISCARD T material::get_param(const std::string& name)
    {
        id_type id = nameHash(name);
        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<T>())
            return static_cast<material_parameter<T>*>(m_parameters[id].get())->get_value();

        log::warn("material {} does not have a parameter named {} of type {}", m_name, name, typeName<T>());
        return T();
    }

    template<typename T>
    void material::set_param(GLint location, const T& value)
    {
        if (!m_idOfLocation.count(location))
            log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<T>());

        id_type id = m_idOfLocation[location];

        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<T>())
            static_cast<material_parameter<T>*>(m_parameters[id].get())->set_value(value);
        else
            log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<T>());
    }

    template<typename T>
    L_NODISCARD T material::get_param(GLint location)
    {
        if (!m_idOfLocation.count(location))
            log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<T>());

        id_type id = m_idOfLocation[location];
        if (m_parameters.count(id) && m_parameters[id]->type() == typeHash<T>())
            return static_cast<material_parameter<T>*>(m_parameters[id].get())->get_value();

        log::warn("material {} does not have a parameter at location {} of type {}", m_name, location, typeName<T>());
        return T();
    }

    template<typename T>
    L_NODISCARD bool material::has_param(GLint location)
    {
        if (!m_idOfLocation.count(location))
            return false;

        id_type id = m_idOfLocation[location];
        return m_parameters.count(id) && m_parameters[id]->type() == typeHash<T>();
    }
#pragma endregion
}


#if !defined(DOXY_EXCLUDE)
namespace std
{
    template<>
    struct hash<legion::rendering::material_handle>
    {
        std::size_t operator()(legion::rendering::material_handle const& handle) const noexcept
        {
            return static_cast<std::size_t>(handle.id);
        }
    };
}
#endif
