#include <rendering/data/shader.hpp>
#include <rendering/util/bindings.hpp>
#include <algorithm>
#include <rendering/shadercompiler/shadercompiler.hpp>

namespace legion::rendering
{
    sparse_map<id_type, shader> ShaderCache::m_shaders;
    async::rw_spinlock ShaderCache::m_shaderLock;

    shader* ShaderCache::get_shader(id_type id)
    {
        async::readonly_guard guard(m_shaderLock);
        return &m_shaders[id];
    }


    void ShaderCache::process_io(shader& shader, id_type id)
    {
        // Clear all values.
        shader.uniforms.clear();
        shader.attributes.clear();

#pragma region uniforms
        // Find the number of active uniforms.
        GLint numActiveUniforms = 0;
        glGetProgramiv(shader.programId, GL_ACTIVE_UNIFORMS, &numActiveUniforms);

        GLint maxUniformNameLength = 0; // Get name buffer length.
        glGetProgramiv(shader.programId, GL_ACTIVE_UNIFORM_MAX_LENGTH, &maxUniformNameLength);

        GLchar* uniformNameBuffer = new GLchar[maxUniformNameLength]; // Create buffer with the right length.

        for (int uniformId = 0; uniformId < numActiveUniforms; uniformId++)
        {
            GLint arraySize = 0; // Use this later for uniform arrays.
            GLenum type = 0;
            GLsizei nameLength = 0;
            glGetActiveUniform(shader.programId, uniformId, (GLsizei)maxUniformNameLength, &nameLength, &arraySize, &type, uniformNameBuffer);

            std::string_view name(uniformNameBuffer, nameLength + 1); // Get string_view of the actual name within the buffer.

            if (name.find('[') != std::string_view::npos) // We don't support uniform arrays yet.
                continue;

            // Get location and create uniform object.
            app::gl_location location = glGetUniformLocation(shader.programId, uniformNameBuffer);
            shader_parameter_base* uniform = nullptr;
            switch (type)
            {
            case GL_SAMPLER_2D:
                uniform = new rendering::uniform<texture_handle>(id, name, type, location);
                break;
            case GL_UNSIGNED_INT:
                uniform = new rendering::uniform<uint>(id, name, type, location);
                break;
            case GL_FLOAT:
                uniform = new rendering::uniform<float>(id, name, type, location);
                break;
            case GL_FLOAT_VEC2:
                uniform = new rendering::uniform<math::vec2>(id, name, type, location);
                break;
            case GL_FLOAT_VEC3:
                uniform = new rendering::uniform<math::vec3>(id, name, type, location);
                break;
            case GL_FLOAT_VEC4:
                uniform = new rendering::uniform<math::vec4>(id, name, type, location);
                break;
            case GL_INT:
                uniform = new rendering::uniform<int>(id, name, type, location);
                break;
            case GL_INT_VEC2:
                uniform = new rendering::uniform<math::ivec2>(id, name, type, location);
                break;
            case GL_INT_VEC3:
                uniform = new rendering::uniform<math::ivec3>(id, name, type, location);
                break;
            case GL_INT_VEC4:
                uniform = new rendering::uniform<math::ivec4>(id, name, type, location);
                break;
            case GL_BOOL:
                uniform = new rendering::uniform<bool>(id, name, type, location);
                break;
            case GL_BOOL_VEC2:
                uniform = new rendering::uniform<math::bvec2>(id, name, type, location);
                break;
            case GL_BOOL_VEC3:
                uniform = new rendering::uniform<math::bvec3>(id, name, type, location);
                break;
            case GL_BOOL_VEC4:
                uniform = new rendering::uniform<math::bvec4>(id, name, type, location);
                break;
            case GL_FLOAT_MAT2:
                uniform = new rendering::uniform<math::mat2>(id, name, type, location);
                break;
            case GL_FLOAT_MAT3:
                uniform = new rendering::uniform<math::mat3>(id, name, type, location);
                break;
            case GL_FLOAT_MAT4:
                uniform = new rendering::uniform<math::mat4>(id, name, type, location);
                break;
            default:
                continue;
            }

            // Insert uniform into the uniform list.
            auto hashid = nameHash(std::string(name));
            shader.uniforms[hashid] = std::unique_ptr<shader_parameter_base>(uniform);
            shader.idOfLocation[location] = hashid;
        }

        delete[] uniformNameBuffer; // Delete name buffer
#pragma endregion
#pragma region attributes
        // Find the number of active attributes.
        GLint numActiveAttribs = 0;
        glGetProgramiv(shader.programId, GL_ACTIVE_ATTRIBUTES, &numActiveAttribs);

        GLint maxAttribNameLength = 0; // Get name buffer length.
        glGetProgramiv(shader.programId, GL_ACTIVE_ATTRIBUTE_MAX_LENGTH, &maxAttribNameLength);

        GLchar* attribNameBuffer = new GLchar[maxAttribNameLength]; // Create buffer with the right length.

        for (int attrib = 0; attrib < numActiveAttribs; ++attrib)
        {
            GLint arraySize = 0; // Use this later for attribute arrays.
            GLenum type = 0;
            GLsizei nameLength = 0;
            glGetActiveAttrib(shader.programId, attrib, (GLsizei)maxAttribNameLength, &nameLength, &arraySize, &type, attribNameBuffer);

            std::string_view name(attribNameBuffer, nameLength + 1); // Get string_view of the actual name within the buffer.

            if (name.find('[') != std::string_view::npos) // We don't support attribute arrays yet.
                continue;

            // Get location and create attribute object
            GLint location = glGetAttribLocation(shader.programId, attribNameBuffer);
            shader.attributes[nameHash(std::string(name).c_str())] = std::unique_ptr<attribute>(new attribute(id, name, type, location));
        }

        delete[] attribNameBuffer;
#pragma endregion
    }

    app::gl_id ShaderCache::compile_shader(GLuint shaderType, cstring source, GLint sourceLength)
    {
        // Create and compile shader.
        app::gl_id shaderId = glCreateShader(shaderType);
        glShaderSource(shaderId, 1, &source, &sourceLength);
        glCompileShader(shaderId);

        // Fetch compile status.
        GLint compilerResult = GL_FALSE;
        glGetShaderiv(shaderId, GL_COMPILE_STATUS, &compilerResult);

        // Check for errors.
        if (!compilerResult)
        {
            // Create message buffer and fetch message.
            GLint infoLogLength;
            glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &infoLogLength);
            char* errorMessage;
            if (infoLogLength > 0)
            {
                errorMessage = new char[infoLogLength + 1];
                glGetShaderInfoLog(shaderId, infoLogLength, nullptr, errorMessage);
                errorMessage[infoLogLength] = '\0';
            }
            else
            {
                errorMessage = "Unknown error, OpenGL context might not have been made current?";
            }
            cstring shaderTypename;
            switch (shaderType)
            {
            case GL_FRAGMENT_SHADER:
                shaderTypename = "fragment";
                break;
            case GL_VERTEX_SHADER:
                shaderTypename = "vertex";
                break;
            case GL_GEOMETRY_SHADER:
                shaderTypename = "geometry";
                break;
            case GL_TESS_CONTROL_SHADER:
                shaderTypename = "tessellation control";
                break;
            case GL_TESS_EVALUATION_SHADER:
                shaderTypename = "tessellation evaluation";
                break;
            case GL_COMPUTE_SHADER:
                shaderTypename = "compute";
                break;
            default:
                shaderTypename = "unknown type";
                break;
            }

            log::error("Error compiling {} shader:\n\t{}", shaderTypename, errorMessage);
            if (infoLogLength > 0)
                delete[] errorMessage; // Delete message.

            glDeleteShader(shaderId); // Delete shader.
            return -1;
        }
        return shaderId;
    }

    bool ShaderCache::load_precompiled(const fs::view& file, shader_ilo& ilo, shader_state& state)
    {
        log::info("Loading precompiled shader: {}", file.get_virtual_path());
        auto result = file.get();
        if (result != common::valid)
            return false;

        auto resource = result.decay();

        if (resource.size() <= 22)
            return false;

        byte_vec data = resource.get();

        std::string_view magic(reinterpret_cast<char*>(data.data()), 19);
        if (magic != "\xabLEGION SHADER\xbb\r\n\x13\n")
            return false;

        auto start = data.cbegin() + 19;
        auto end = data.cend();

        while (start != end)
        {
            GLenum shaderType;
            retrieveBinaryData(shaderType, start);

            switch (shaderType)
            {
            case 0:
            {
                std::vector<GLenum> rawState;
                retrieveBinaryData(rawState, start);
                if (rawState.size() % 2 != 0)
                    return false;

                for (int i = 0; i < rawState.size(); i += 2)
                {
                    state[rawState[i]] = rawState[i + 1];
                }
            }
            break;
            case GL_VERTEX_SHADER:
            case GL_FRAGMENT_SHADER:
            case GL_GEOMETRY_SHADER:
            {
                std::string source;
                retrieveBinaryData(source, start);
                ilo.push_back(std::make_pair(shaderType, source));
            }
            break;
            default:
                return false;
            }
        }
        return true;
    }

    void ShaderCache::store_precompiled(const fs::view& file, const shader_ilo& ilo, const shader_state& state)
    {
        auto result = file.get_extension();
        if (result != common::valid)
            return;

        fs::view precompiled("");
        if (result.decay() == ".shil" || result.decay().empty())
            precompiled = file;
        else
            precompiled = file / ".." / (file.get_filestem().decay() + ".shil");

        if (precompiled.is_valid(true) && precompiled.file_info().can_be_written)
        {
            fs::basic_resource resource(nullptr);
            byte_vec& data = resource.get();

            std::string magic = "\xabLEGION SHADER\xbb\r\n\x13\n";
            for (auto item : magic)
                data.push_back(item);

            GLenum stateType = 0;
            appendBinaryData(&stateType, data);

            std::vector<GLenum> rawState;
            for (auto& [key, value] : state)
            {
                rawState.push_back(key);
                rawState.push_back(value);
            }

            appendBinaryData(&rawState, data);

            for (auto& [shaderType, source] : ilo)
            {
                appendBinaryData(&shaderType, data);
                appendBinaryData(&source, data);
            }

            auto garbage = precompiled.set(resource);
        }

    }

    shader_handle ShaderCache::create_invalid_shader(const fs::view& file, shader_import_settings settings)
    {
        { // Check if the shader already exists.
            async::readonly_guard guard(m_shaderLock);
            if (m_shaders.contains(invalid_id) && m_shaders[invalid_id].programId != 0)
            {
                log::debug("Shader invalid already exists, existing shader will be returned instead.");
                return { invalid_id };
            }
        }

        shader_state state;
        shader_ilo shaders;

        auto result = file.get_extension();
        if (result != common::valid)
            return invalid_shader_handle;

        bool compiledFromScratch = false;

        if (result.decay().empty() || result.decay() == ".shil")
        {
            if (!load_precompiled(file, shaders, state))
                return invalid_shader_handle;
        }
        else
        {
            switch (settings.usePrecompiledIfAvailable)
            {
            case true:
            {
                auto precompiled = file / ".." / (file.get_filestem().decay() + ".shil");

                if (precompiled.is_valid(true))
                {
                    auto traits = precompiled.file_info();
                    if (traits.is_file && traits.can_be_read)
                    {
                        if (load_precompiled(precompiled, shaders, state))
                            break;
                    }
                }
            }
            L_FALLTHROUGH;
            default:
            {
                ShaderCompiler::setErrorCallback([](const std::string& errormsg, log::severity severity)
                    {
                        log::println(severity, errormsg);
                    });

                byte compilerSettings = 0;
                compilerSettings |= settings.api;
                if (settings.debug)
                    compilerSettings |= shader_compiler_options::debug;
                if (settings.low_power)
                    compilerSettings |= shader_compiler_options::low_power;

                if (!ShaderCompiler::process(file, compilerSettings, shaders, state, detail::get_default_defines()))
                    return invalid_shader_handle;

                compiledFromScratch = true;
            }
            break;
            }
        }

        if (shaders.empty())
            return invalid_shader_handle;

        shader shader;
        shader.name = "invalid";
        shader.state = state;

        GLenum blendSrc = GL_SRC_ALPHA, blendDst = GL_ONE_MINUS_SRC_ALPHA;

        for (auto& [func, param] : state)
        {
            switch (func)
            {
            case GL_DEPTH_TEST:
            {
                if (param == GL_FALSE)
                {
                    glDisable(func);
                    continue;
                }

                glEnable(func);
                glDepthFunc(param);
            }
            break;
            case GL_CULL_FACE:
            {
                if (param == GL_FALSE)
                {
                    glDisable(func);
                    continue;
                }

                glEnable(func);
                glCullFace(param);
            }
            break;
            case GL_BLEND:
            {
                blendSrc = param;
                blendDst = param;
            }
            case GL_BLEND_SRC:
            {
                blendSrc = param;
            }
            case GL_BLEND_DST:
            {
                blendDst = param;
            }
            break;
            case GL_DITHER:
            {
                if (param == GL_TRUE)
                    glEnable(GL_DITHER);
                else
                    glDisable(GL_DITHER);
            }
            break;
            default:
                break;
            }
        }

        if (blendSrc == GL_FALSE || blendDst == GL_FALSE)
        {
            glDisable(GL_BLEND);
        }
        else
        {
            glEnable(GL_BLEND);
            glBlendFunc(blendSrc, blendDst);
        }

        shader.programId = glCreateProgram();

        std::vector<app::gl_id> shaderIds;

        for (auto& [shaderType, shaderIL] : shaders)
        {
            auto shaderId = compile_shader(shaderType, shaderIL.c_str(), shaderIL.size());

            if (shaderId == (app::gl_id)-1)
            {
                auto v = common::split_string_at<'\n'>(shaderIL);

                std::string output;
                for (int i = 0; i < v.size(); i++)
                {
                    output += std::to_string(i + 1) + "\t| " + v[i] + "\n";
                }

                log::error("Error occurred in shader: invalid\n{}", output);

                for (auto id : shaderIds)
                {
                    glDetachShader(shader.programId, id);
                    glDeleteShader(id);
                }

                glDeleteProgram(shader.programId);
                return invalid_shader_handle;
            }

            glAttachShader(shader.programId, shaderId);
            shaderIds.push_back(shaderId);
        }

        glLinkProgram(shader.programId);

        GLint linkStatus;
        glGetProgramiv(shader.programId, GL_LINK_STATUS, &linkStatus);

        if (!linkStatus)
        {
            GLint infoLogLength;
            glGetProgramiv(shader.programId, GL_INFO_LOG_LENGTH, &infoLogLength);

            char* errorMessage = new char[infoLogLength + 1];
            glGetProgramInfoLog(shader.programId, infoLogLength, nullptr, errorMessage);

            log::error("Error linking invalid shader:\n\t{}", errorMessage);
            delete[] errorMessage;

            for (auto& [shaderType, shaderIL] : shaders)
            {
                cstring shaderTypename;
                switch (shaderType)
                {
                case GL_FRAGMENT_SHADER:
                    shaderTypename = "fragment";
                    break;
                case GL_VERTEX_SHADER:
                    shaderTypename = "vertex";
                    break;
                case GL_GEOMETRY_SHADER:
                    shaderTypename = "geometry";
                    break;
                case GL_TESS_CONTROL_SHADER:
                    shaderTypename = "tessellation control";
                    break;
                case GL_TESS_EVALUATION_SHADER:
                    shaderTypename = "tessellation evaluation";
                    break;
                case GL_COMPUTE_SHADER:
                    shaderTypename = "compute";
                    break;
                default:
                    shaderTypename = "unknown type";
                    break;
                }

                log::error("{}:\n\n{}\n\n", shaderTypename, shaderIL);

            }

            for (auto& id : shaderIds)
            {
                glDetachShader(shader.programId, id);
                glDeleteShader(id);
            }

            glDeleteProgram(shader.programId);

            return invalid_shader_handle;
        }

        glValidateProgram(shader.programId);

        GLint validationStatus;
        glGetProgramiv(shader.programId, GL_VALIDATE_STATUS, &validationStatus);

        if (!validationStatus)
        {
            GLint infoLogLength;
            glGetProgramiv(shader.programId, GL_INFO_LOG_LENGTH, &infoLogLength);

            char* errorMessage = new char[infoLogLength + 1];
            glGetProgramInfoLog(shader.programId, infoLogLength, nullptr, errorMessage);

            log::error("Error validating invalid shader:\n\t{}", errorMessage);
            delete[] errorMessage;

            for (auto& shaderId : shaderIds)
            {
                glDetachShader(shader.programId, shaderId);
                glDeleteShader(shaderId);
            }

            glDeleteProgram(shader.programId);

            return invalid_shader_handle;
        }

        process_io(shader, invalid_id);

        {
            async::readwrite_guard guard(m_shaderLock);
            m_shaders[invalid_id] = std::move(shader);
        }

        if (compiledFromScratch && settings.storePrecompiled)
            store_precompiled(file, shaders, state);

        return { invalid_id };
    }

    shader_handle ShaderCache::create_shader(const std::string& name, const fs::view& file, shader_import_settings settings)
    {
        // Get the id of the new shader.
        id_type id = nameHash(name);

        { // Check if the shader already exists.
            async::readonly_guard guard(m_shaderLock);

            if (!m_shaders.contains(invalid_id) || m_shaders[invalid_id].programId == 0)
            {
                create_invalid_shader(fs::view("engine://shaders/invalid.shs"));
            }

            if (m_shaders.contains(id))
            {
                log::debug("Shader {} already exists, existing shader will be returned instead.", name);
                return { id };
            }
        }

        shader_state state;
        shader_ilo shaders;

        auto result = file.get_extension();
        if (result != common::valid)
            return invalid_shader_handle;

        bool compiledFromScratch = false;

        if (result.decay().empty() || result.decay() == ".shil")
        {
            if (!load_precompiled(file, shaders, state))
                return invalid_shader_handle;
        }
        else
        {
            switch (settings.usePrecompiledIfAvailable)
            {
            case true:
            {
                auto precompiled = file / ".." / (file.get_filestem().decay() + ".shil");

                if (precompiled.is_valid(true))
                {
                    auto traits = precompiled.file_info();
                    if (traits.is_file && traits.can_be_read)
                    {
                        if (load_precompiled(precompiled, shaders, state))
                            break;
                    }
                }
            }
            L_FALLTHROUGH;
            default:
            {
                ShaderCompiler::setErrorCallback([](const std::string& errormsg, log::severity severity)
                    {
                        log::println(severity, errormsg);
                    });

                byte compilerSettings = 0;
                compilerSettings |= settings.api;
                if (settings.debug)
                    compilerSettings |= shader_compiler_options::debug;
                if (settings.low_power)
                    compilerSettings |= shader_compiler_options::low_power;

                if (!ShaderCompiler::process(file, compilerSettings, shaders, state, detail::get_default_defines()))
                    return invalid_shader_handle;

                compiledFromScratch = true;
            }
            break;
            }
        }

        if (shaders.empty())
            return invalid_shader_handle;

        shader shader;
        shader.name = name;
        shader.state = state;

        GLenum blendSrc = GL_SRC_ALPHA, blendDst = GL_ONE_MINUS_SRC_ALPHA;

        for (auto& [func, param] : state)
        {
            switch (func)
            {
            case GL_DEPTH_TEST:
            {
                if (param == GL_FALSE)
                {
                    glDisable(func);
                    continue;
                }

                glEnable(func);
                glDepthFunc(param);
            }
            break;
            case GL_CULL_FACE:
            {
                if (param == GL_FALSE)
                {
                    glDisable(func);
                    continue;
                }

                glEnable(func);
                glCullFace(param);
            }
            break;
            case GL_BLEND:
            {
                blendSrc = param;
                blendDst = param;
            }
            case GL_BLEND_SRC:
            {
                blendSrc = param;
            }
            case GL_BLEND_DST:
            {
                blendDst = param;
            }
            break;
            case GL_DITHER:
            {
                if (param == GL_TRUE)
                    glEnable(GL_DITHER);
                else
                    glDisable(GL_DITHER);
            }
            break;
            default:
                break;
            }
        }

        if (blendSrc == GL_FALSE || blendDst == GL_FALSE)
        {
            glDisable(GL_BLEND);
        }
        else
        {
            glEnable(GL_BLEND);
            glBlendFunc(blendSrc, blendDst);
        }

        shader.programId = glCreateProgram();

        std::vector<app::gl_id> shaderIds;

        for (auto& [shaderType, shaderIL] : shaders)
        {
            auto shaderId = compile_shader(shaderType, shaderIL.c_str(), shaderIL.size());

            if (shaderId == (app::gl_id)-1)
            {
                auto v = common::split_string_at<'\n'>(shaderIL);

                std::string output;
                for (int i = 0; i < v.size(); i++)
                {
                    output += std::to_string(i + 1) + "\t| " + v[i] + "\n";
                }

                log::error("Error occurred in shader: {}\n{}", name, output);

                for (auto id : shaderIds)
                {
                    glDetachShader(shader.programId, id);
                    glDeleteShader(id);
                }

                glDeleteProgram(shader.programId);
                return invalid_shader_handle;
            }

            glAttachShader(shader.programId, shaderId);
            shaderIds.push_back(shaderId);
        }

        glLinkProgram(shader.programId);

        GLint linkStatus;
        glGetProgramiv(shader.programId, GL_LINK_STATUS, &linkStatus);

        if (!linkStatus)
        {
            GLint infoLogLength;
            glGetProgramiv(shader.programId, GL_INFO_LOG_LENGTH, &infoLogLength);

            char* errorMessage = new char[infoLogLength + 1];
            glGetProgramInfoLog(shader.programId, infoLogLength, nullptr, errorMessage);

            log::error("Error linking {} shader:\n\t{}", name, errorMessage);
            delete[] errorMessage;

            for (auto& [shaderType, shaderIL] : shaders)
            {
                cstring shaderTypename;
                switch (shaderType)
                {
                case GL_FRAGMENT_SHADER:
                    shaderTypename = "fragment";
                    break;
                case GL_VERTEX_SHADER:
                    shaderTypename = "vertex";
                    break;
                case GL_GEOMETRY_SHADER:
                    shaderTypename = "geometry";
                    break;
                case GL_TESS_CONTROL_SHADER:
                    shaderTypename = "tessellation control";
                    break;
                case GL_TESS_EVALUATION_SHADER:
                    shaderTypename = "tessellation evaluation";
                    break;
                case GL_COMPUTE_SHADER:
                    shaderTypename = "compute";
                    break;
                default:
                    shaderTypename = "unknown type";
                    break;
                }

                log::error("{}:\n\n{}\n\n", shaderTypename, shaderIL);

            }

            for (auto& id : shaderIds)
            {
                glDetachShader(shader.programId, id);
                glDeleteShader(id);
            }

            glDeleteProgram(shader.programId);

            return invalid_shader_handle;
        }

        glValidateProgram(shader.programId);

        GLint validationStatus;
        glGetProgramiv(shader.programId, GL_VALIDATE_STATUS, &validationStatus);

        if (!validationStatus)
        {
            GLint infoLogLength;
            glGetProgramiv(shader.programId, GL_INFO_LOG_LENGTH, &infoLogLength);

            char* errorMessage = new char[infoLogLength + 1];
            glGetProgramInfoLog(shader.programId, infoLogLength, nullptr, errorMessage);

            log::error("Error validating {} shader:\n\t{}", name, errorMessage);
            delete[] errorMessage;

            for (auto& shaderId : shaderIds)
            {
                glDetachShader(shader.programId, shaderId);
                glDeleteShader(shaderId);
            }

            glDeleteProgram(shader.programId);

            return invalid_shader_handle;
        }

        process_io(shader, id);

        {
            async::readwrite_guard guard(m_shaderLock);
            m_shaders.insert(id, std::move(shader));
        }

        if (compiledFromScratch && settings.storePrecompiled)
            store_precompiled(file, shaders, state);

        return { id };
    }

    shader_handle ShaderCache::create_shader(const fs::view& file, shader_import_settings settings)
    {
        return create_shader(file.get_filename(), file, settings);
    }

    shader_handle ShaderCache::get_handle(const std::string& name)
    {
        id_type id = nameHash(name);

        async::readonly_guard guard(m_shaderLock);

        if (!m_shaders.contains(invalid_id) || m_shaders[invalid_id].programId == 0)
        {
            create_invalid_shader(fs::view("engine://shaders/invalid.shs"));
        }

        if (!m_shaders.contains(id))
            return invalid_shader_handle;
        else
            return { id };
    }

    shader_handle ShaderCache::get_handle(id_type id)
    {
        async::readonly_guard guard(m_shaderLock);

        if (!m_shaders.contains(invalid_id) || m_shaders[invalid_id].programId == 0)
        {
            create_invalid_shader(fs::view("engine://shaders/invalid.shs"));
        }

        if (!m_shaders.contains(id))
            return invalid_shader_handle;
        else
            return { id };
    }

    GLuint shader_handle::get_uniform_block_index(const std::string& name) const
    {
        return ShaderCache::get_shader(id)->get_uniform_block_index(name);
    }

    void shader_handle::bind_uniform_block(GLuint uniformBlockIndex, GLuint uniformBlockBinding) const
    {
        ShaderCache::get_shader(id)->bind_uniform_block(uniformBlockIndex, uniformBlockBinding);
    }

    std::string shader_handle::get_name() const
    {
        return ShaderCache::get_shader(id)->name;
    }

    std::vector<std::tuple<std::string, GLint, GLenum>> shader_handle::get_uniform_info() const
    {
        OPTICK_EVENT();
        return ShaderCache::get_shader(id)->get_uniform_info();
    }

    attribute shader_handle::get_attribute(const std::string& name)
    {
        return ShaderCache::get_shader(id)->get_attribute(name);
    }

    attribute shader_handle::get_attribute(id_type attributeId)
    {
        return ShaderCache::get_shader(id)->get_attribute(attributeId);
    }

    void shader_handle::bind()
    {
        ShaderCache::get_shader(id)->bind();
    }

    void shader_handle::release()
    {
        glUseProgram(0);
    }

    void shader::bind()
    {
        GLenum blendSrc = GL_SRC_ALPHA, blendDst = GL_ONE_MINUS_SRC_ALPHA;

        for (auto& [func, param] : state)
        {
            switch (func)
            {
            case GL_DEPTH_TEST:
            {
                if (param == GL_FALSE)
                {
                    glDisable(func);
                    continue;
                }

                glEnable(func);
                glDepthFunc(param);
            }
            break;
            case GL_CULL_FACE:
            {
                if (param == GL_FALSE)
                {
                    glDisable(func);
                    continue;
                }

                glEnable(func);
                glCullFace(param);
            }
            break;
            case GL_BLEND:
            {
                blendSrc = param;
                blendDst = param;
            }
            case GL_BLEND_SRC:
            {
                blendSrc = param;
            }
            case GL_BLEND_DST:
            {
                blendDst = param;
            }
            break;
            case GL_DITHER:
            {
                if (param == GL_TRUE)
                    glEnable(GL_DITHER);
                else
                    glDisable(GL_DITHER);
            }
            break;
            default:
                break;
            }
        }

        if (blendSrc == GL_FALSE || blendDst == GL_FALSE)
        {
            glDisable(GL_BLEND);
        }
        else
        {
            glEnable(GL_BLEND);
            glBlendFunc(blendSrc, blendDst);
        }

        glUseProgram(programId);
    }

    void shader::release()
    {
        glUseProgram(0);
    }

    GLuint shader::get_uniform_block_index(const std::string& name) const
    {
        return glGetUniformBlockIndex(programId, name.c_str());
    }

    void shader::bind_uniform_block(GLuint uniformBlockIndex, GLuint uniformBlockBinding) const
    {
        glUniformBlockBinding(programId, uniformBlockIndex, uniformBlockBinding);
    }

}
