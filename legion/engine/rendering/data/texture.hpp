#pragma once
#include <application/application.hpp>

/**
 * @file texture.hpp
 */

namespace legion::rendering
{
    enum struct texture_type : GLenum
    {
        one_dimensional = GL_TEXTURE_1D,
        two_dimensional = GL_TEXTURE_2D,
        three_dimensional = GL_TEXTURE_3D,
        array_1D = GL_TEXTURE_1D_ARRAY,
        array_2D = GL_TEXTURE_2D_ARRAY,
        rectangle = GL_TEXTURE_RECTANGLE,
        cube_map_positive_x = GL_TEXTURE_CUBE_MAP_POSITIVE_X,
        cube_map_negative_x = GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
        cube_map_positive_y = GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
        cube_map_negative_y = GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
        cube_map_positive_z = GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
        cube_map_negative_z = GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
    };

    enum struct texture_format : GLint
    {
        depth = GL_DEPTH_COMPONENT,
        stencil = GL_STENCIL_INDEX,
        depth_stencil = GL_DEPTH24_STENCIL8,
        rgba_hdr = GL_RGBA32F,
        rgb_hdr = GL_RGB32F,
        red = GL_RED,
        rg = GL_RG,
        rgb = GL_RGB,
        rgba = GL_RGBA,
        bgr = GL_BGR,
        bgra = GL_BGRA,
        red_int = GL_RED_INTEGER,
        rg_int = GL_RG_INTEGER,
        rgb_int = GL_RGB_INTEGER,
        rgba_int = GL_RGBA_INTEGER,
        bgr_int = GL_BGR_INTEGER,
        bgra_int = GL_BGRA_INTEGER,
    };

    /**@brief Internal channel layout of the colors.
     * @ref legion::core::image_components
     */
    using texture_components = image_components;

    /**@brief Utility array for converting component count to GLenum. (components_to_format[4] = GL_RGBA)
     */
    constexpr GLenum components_to_format[] = { 0, GL_RED, GL_RG, GL_RGB, GL_RGBA, GL_DEPTH_COMPONENT, GL_STENCIL_INDEX, GL_DEPTH_STENCIL };

    /**@brief Utility array for converting data size to GLenum. (channels_to_glenum[sizeof(byte)] = GL_UNSIGNED_BYTE)
     */
    constexpr GLenum channels_to_glenum[] = { GL_UNSIGNED_INT_24_8, GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT, 0, GL_FLOAT };

    enum struct texture_mipmap : GLint
    {
        nearest = GL_NEAREST,
        linear = GL_LINEAR
    };

    enum struct texture_wrap : GLint
    {
        edge_clamp = GL_CLAMP_TO_EDGE,
        border_clamp = GL_CLAMP_TO_BORDER,
        mirror = GL_MIRRORED_REPEAT,
        repeat = GL_REPEAT,
        mirror_then_clamp = GL_MIRROR_CLAMP_TO_EDGE
    };

    /**@class texture_data
     * @brief Raw texture representation.
     */
    struct texture_data
    {
        math::ivec2 size;
        texture_type type;

        std::vector<math::color> pixels;
    };

    /**@class texture
     * @brief Struct containing all the data needed for rendering.
     */
    struct texture
    {
        std::string name;
        app::gl_id textureId = invalid_id;

        texture_components channels;
        texture_type type;
        texture_format format;
        channel_format fileFormat;

        static void to_resource(fs::basic_resource* resource, const texture& value);
        static void from_resource(texture* value, const fs::basic_resource& resource);
        math::ivec2 size() const;
        void resize(math::ivec2 size) const;
    };

    /**@class texture_handle
     * @brief Save to pass around handle to a texture in the texture cache.
     */
    struct texture_handle
    {
        id_type id = invalid_id;

        texture_data get_data() const;
        const texture& get_texture() const;
        bool operator==(const texture_handle& other) const { return id == other.id; }
        operator id_type() const { return id; }
    };

    /**@brief Default invalid texture handle.
     */
    constexpr texture_handle invalid_texture_handle { invalid_id };

    /**@class texture_import_settings
     * @brief Data structure to parameterize the texture import process.
     */
    struct texture_import_settings
    {
        texture_type type;
        channel_format fileFormat;
        texture_format intendedFormat;
        texture_components components;
        bool flipVertical;
        bool generateMipmaps;
        texture_mipmap min;
        texture_mipmap mag;
        texture_wrap wrapR;
        texture_wrap wrapS;
        texture_wrap wrapT;
    };

    /**@brief Default texture import settings.
     */
    constexpr texture_import_settings default_texture_settings{
        texture_type::two_dimensional, channel_format::eight_bit, texture_format::rgba,
        texture_components::rgba, true, true, texture_mipmap::linear, texture_mipmap::linear,
        texture_wrap::repeat, texture_wrap::repeat, texture_wrap::repeat };

    /**@class TextureCache
     * @brief Data cache for loading, storing and managing textures.
     */
    class TextureCache
    {
        friend class renderer;
        friend struct texture_handle;
    private:
        static sparse_map<id_type, texture> m_textures;
        static async::rw_spinlock m_textureLock;

        static const texture& get_texture(id_type id);
        static texture_data get_data(id_type id);
        static texture_handle m_invalidTexture;
    public:
        /**@brief Create a new texture and load it from a file if a texture with the same name doesn't exist yet.
         * @param name Identifying name for the texture.
         * @param file File to load from.
         * @param settings Settings to pass on to the import pipeline.
         * @return texture_handle A valid handle to the newly created texture if it succeeds, invalid_texture_handle if it fails.
         */
        static texture_handle create_texture(const std::string& name, const fs::view& file, texture_import_settings settings = default_texture_settings);
        static texture_handle create_texture(const fs::view& file, texture_import_settings settings = default_texture_settings);
        static texture_handle create_texture(const std::string& name, math::ivec2 size, texture_import_settings settings = default_texture_settings);

        /**@brief Create a new texture from an image if a texture with the same name doesn't exist yet.
         * @param name Name of the image and identifying name for the texture.
         * @param settings Settings to pass on to the import pipeline.
         * @return texture_handle A valid handle to the newly created texture if it succeeds, invalid_texture_handle if it fails.
         */
        static texture_handle create_texture_from_image(const std::string& name, texture_import_settings settings = default_texture_settings);

        /**@brief Create a new texture from an image if a texture with the same name doesn't exist yet.
         * @param image_handle Image to load from. The identifying name for the texture will be the same as the name of the image.
         * @param settings Settings to pass on to the import pipeline.
         * @return texture_handle A valid handle to the newly created texture if it succeeds, invalid_texture_handle if it fails.
         */
        static texture_handle create_texture_from_image(image_handle image, texture_import_settings settings = default_texture_settings);

        /**@brief Returns a handle to a texture with a certain name. Will return invalid_texture_handle if the requested texture doesn't exist.
         */
        static texture_handle get_handle(const std::string& name);

        /**@brief Returns a handle to a texture with a certain name. Will return invalid_texture_handle if the requested texture doesn't exist.
         * @param id Name hash
         */
        static texture_handle get_handle(id_type id);
    };
}
