#include <rendering/data/texture.hpp>

namespace legion::rendering
{
    void texture::to_resource(fs::basic_resource* resource, const texture& value)
    {
        OPTICK_EVENT();
        resource->clear();
        appendBinaryData(&value.textureId, resource->get());
        appendBinaryData(&value.channels, resource->get());
        appendBinaryData(&value.type, resource->get());
    }

    void texture::from_resource(texture* value, const fs::basic_resource& resource)
    {
        OPTICK_EVENT();
        byte_vec::const_iterator start = resource.begin();
        retrieveBinaryData(value->textureId, start);
        retrieveBinaryData(value->channels, start);
        retrieveBinaryData(value->type, start);
    }

    math::ivec2 texture::size() const
    {
        OPTICK_EVENT();
        math::ivec2 texSize;
        glBindTexture(static_cast<GLenum>(type), textureId);
        glGetTexLevelParameteriv(static_cast<GLenum>(type), 0, GL_TEXTURE_WIDTH, &texSize.x);
        glGetTexLevelParameteriv(static_cast<GLenum>(type), 0, GL_TEXTURE_HEIGHT, &texSize.y);
        glBindTexture(static_cast<GLenum>(type), 0);
        return texSize;
    }

    void texture::resize(math::ivec2 newSize) const
    {
        OPTICK_EVENT();
        glBindTexture(static_cast<GLenum>(type), textureId);
        glTexImage2D(
            static_cast<GLenum>(type),
            0,
            static_cast<GLint>(format),
            newSize.x,
            newSize.y,
            0,
            components_to_format[static_cast<int>(channels)],
            channels_to_glenum[static_cast<uint>(fileFormat)],
            NULL);
        glBindTexture(static_cast<GLenum>(type), 0);
    }

    sparse_map<id_type, texture> TextureCache::m_textures;
    async::rw_spinlock TextureCache::m_textureLock;
    texture_handle TextureCache::m_invalidTexture;

    texture_data texture_handle::get_data() const
    {
        OPTICK_EVENT();
        return TextureCache::get_data(id);
    }

    const texture& texture_handle::get_texture() const
    {
        OPTICK_EVENT();
        return TextureCache::get_texture(id);
    }

    const texture& TextureCache::get_texture(id_type id)
    {
        OPTICK_EVENT();
        if (m_invalidTexture.id == invalid_id)
            m_invalidTexture = create_texture("invalid texture", fs::view("engine://resources/invalid/missing"));

        async::readonly_guard guard(m_textureLock);
        if (id == invalid_id)
            return m_textures[nameHash("invalid texture")];
        return m_textures[id];
    }

    texture_data TextureCache::get_data(id_type id)
    {
        OPTICK_EVENT();
        texture texture;
        if (m_invalidTexture.id == invalid_id)
            m_invalidTexture = create_texture("invalid texture", fs::view("engine://resources/invalid/missing"));

        {
            async::readonly_guard guard(m_textureLock);
            if (id == invalid_id)
                texture = m_textures[nameHash("invalid texture")];
            else
                texture = m_textures[id];
        }
        texture_data data{};
        math::ivec2 texSize = texture.size();
        data.size.x = texSize.x;
        data.size.y = texSize.y;
        data.type = texture.type;
        data.pixels.resize(data.size.x * data.size.y);
        glBindTexture(static_cast<GLenum>(data.type), texture.textureId);
        glGetTexImage(static_cast<GLenum>(data.type), 0, components_to_format[static_cast<int>(texture.channels)], GL_FLOAT, data.pixels.data());
        glBindTexture(static_cast<GLenum>(data.type), 0);
        return data;
    }

    texture_handle TextureCache::create_texture(const std::string& name, const fs::view& file, texture_import_settings settings)
    {
        OPTICK_EVENT();
        if (m_invalidTexture.id == invalid_id)
        {
            m_invalidTexture.id = 1;
            m_invalidTexture = create_texture("invalid texture", fs::view("engine://resources/invalid/missing"));
        }

        id_type id = nameHash(name);

        {
            async::readonly_guard guard(m_textureLock);
            if (m_textures.contains(id))
                return { id };
        }

        if (!file.is_valid() || !file.file_info().is_file)
            return invalid_texture_handle;

        auto result = fs::AssetImporter::tryLoad<texture>(file, settings);

        if (result != common::valid)
            return invalid_texture_handle;

        {
            async::readwrite_guard guard(m_textureLock);
            m_textures.insert(id, result);
            m_textures.at(id).name = name;
        }
        log::debug("Created texture {} with file: {}", name, file.get_filename().decay());

        return { id };
    }

    texture_handle TextureCache::create_texture(const fs::view& file, texture_import_settings settings)
    {
        OPTICK_EVENT();
        return create_texture(file.get_filename(), file, settings);
    }

    texture_handle TextureCache::create_texture(const std::string& name, math::ivec2 size, texture_import_settings settings)
    {
        OPTICK_EVENT();
        id_type id = nameHash(name);
        {
            async::readonly_guard guard(m_textureLock);
            if (m_textures.contains(id))
                return { id };
        }

        texture texture{};
        texture.type = settings.type;
        texture.name = name;
        // Allocate and bind the texture.
        glGenTextures(1, &texture.textureId);
        glBindTexture(static_cast<GLenum>(settings.type), texture.textureId);

        // Handle mips
        if (settings.generateMipmaps)
        {
            glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_MIN_FILTER, static_cast<GLint>(settings.min));
            glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_MAG_FILTER, static_cast<GLint>(settings.mag));
        }

        // Handle wrapping behavior.
        glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_WRAP_R, static_cast<GLint>(settings.wrapR));
        glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_WRAP_S, static_cast<GLint>(settings.wrapS));
        glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_WRAP_T, static_cast<GLint>(settings.wrapT));

        texture.channels = settings.components;
        texture.format = settings.intendedFormat;
        texture.fileFormat = settings.fileFormat;

        // Construct the texture using the loaded data.
        glTexImage2D(
            static_cast<GLenum>(settings.type),
            0,
            static_cast<GLint>(settings.intendedFormat),
            size.x,
            size.y,
            0,
            components_to_format[static_cast<int>(settings.components)],
            channels_to_glenum[static_cast<uint>(settings.fileFormat)],
            NULL);

        glBindTexture(static_cast<GLenum>(settings.type), 0);

        {
            async::readwrite_guard guard(m_textureLock);
            m_textures.insert(id, texture);
        }

        log::debug("Created blank texture of size {}", size);

        return { id };

    }


    texture_handle TextureCache::create_texture_from_image(const std::string& name, texture_import_settings settings)
    {
        OPTICK_EVENT();
        if (m_invalidTexture.id == invalid_id)
            m_invalidTexture = create_texture("invalid texture", fs::view("engine://resources/invalid/missing"));

        image_handle image = ImageCache::get_handle(name);
        if (image == invalid_image_handle)
        {
            log::warn("Image {} doesn't exist.", name);
            return invalid_texture_handle;
        }

        return create_texture_from_image(image, settings);
    }

    texture_handle TextureCache::create_texture_from_image(image_handle image, texture_import_settings settings)
    {
        OPTICK_EVENT();
        if (m_invalidTexture.id == invalid_id)
            m_invalidTexture = create_texture("invalid texture", fs::view("engine://resources/invalid/missing"));

        if (image == invalid_image_handle)
        {
            log::warn("Tried to create a texture with an invalid image");
            return invalid_texture_handle;
        }

        id_type id = image.id;

        {
            async::readonly_guard guard(m_textureLock);
            if (m_textures.contains(id))
                return { id };
        }

        texture texture{};
        texture.type = settings.type;

        // Allocate and bind the texture.
        glGenTextures(1, &texture.textureId);
        glBindTexture(static_cast<GLenum>(settings.type), texture.textureId);

        // Handle mips
        if (settings.generateMipmaps)
        {
            glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_MIN_FILTER, static_cast<GLint>(settings.min));
            glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_MAG_FILTER, static_cast<GLint>(settings.mag));
        }

        // Handle wrapping behavior.
        glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_WRAP_R, static_cast<GLint>(settings.wrapR));
        glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_WRAP_S, static_cast<GLint>(settings.wrapS));
        glTexParameteri(static_cast<GLenum>(settings.type), GL_TEXTURE_WRAP_T, static_cast<GLint>(settings.wrapT));

        auto [lock, img] = image.get_raw_image();
        {
            async::readonly_guard guard(lock);
            texture.channels = img.components;
            texture.name = img.name;

            // Construct the texture using the loaded data.
            glTexImage2D(
                static_cast<GLenum>(settings.type),
                0,
                static_cast<GLint>(settings.intendedFormat),
                img.size.x,
                img.size.y,
                0,
                components_to_format[static_cast<int>(img.components)],
                channels_to_glenum[static_cast<uint>(img.format)],
                img.get_raw_data<void>());
        }

        // Generate mips.
        if (settings.generateMipmaps)
            glGenerateMipmap(static_cast<GLenum>(settings.type));

        glBindTexture(static_cast<GLenum>(settings.type), 0);

        log::debug("Created texture from image {}", texture.name);

        {
            async::readwrite_guard guard(m_textureLock);
            m_textures.insert(id, std::move(texture));
        }

        return { id };
    }

    texture_handle TextureCache::get_handle(const std::string& name)
    {
        OPTICK_EVENT();
        if (m_invalidTexture.id == invalid_id)
            m_invalidTexture = create_texture("invalid texture", fs::view("engine://resources/invalid/missing"));

        id_type id = nameHash(name);
        async::readonly_guard guard(m_textureLock);
        if (m_textures.contains(id))
            return { id };
        return invalid_texture_handle;
    }

    texture_handle TextureCache::get_handle(id_type id)
    {
        OPTICK_EVENT();
        if (m_invalidTexture.id == invalid_id)
            m_invalidTexture = create_texture("invalid texture", fs::view("engine://resources/invalid/missing"));

        async::readonly_guard guard(m_textureLock);
        if (m_textures.contains(id))
            return { id };
        return invalid_texture_handle;
    }

}
