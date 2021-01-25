#define STBI_MAX_DIMENSIONS 16384
#define STBI_NO_GIF
#define STB_IMAGE_IMPLEMENTATION
#if !defined(DOXY_EXCLUDE)
#include <stb_image.h>
#endif

#include <core/data/importers/image_importers.hpp>

namespace legion::core
{
    common::result_decay_more<image, fs_error> stb_image_loader::load(const filesystem::basic_resource& resource, image_import_settings&& settings)
    {
        OPTICK_EVENT();
        using common::Err, common::Ok;
        // Decay overloads the operator of ok_type and operator== for valid_t.
        using decay = common::result_decay_more<image, fs_error>;

        // Prefetch data from the resource.
        const byte_vec& data = resource.get();

        // Setup stb_image settings.
        stbi_set_flip_vertically_on_load(settings.flipVertical);

        // Create image object.
        image image{};

        // Throwaway temporary storage for the original components in the image that we're loading. (Everything gets converted to the components specified in the settings anyways.)
        image_components components = image_components::grey;

        // Pointer to the start of the data array created by stb_image. It needs to be void because it could either be filled with bytes, ushorts, or floats.
        void* imageData;
        size_type dataSize;

        // Load the image data using stb_image.
        switch (settings.fileFormat)
        {
        default: [[fallthrough]];
        case channel_format::eight_bit:
        {
            imageData = stbi_load_from_memory(data.data(), data.size(), &image.size.x, &image.size.y, reinterpret_cast<int*>(&components), static_cast<int>(settings.components));
            dataSize = image.size.x * image.size.y * static_cast<int>(settings.components) * sizeof(byte);
            break;
        }
        case channel_format::sixteen_bit:
        {
            imageData = stbi_load_16_from_memory(data.data(), data.size(), &image.size.x, &image.size.y, reinterpret_cast<int*>(&components), static_cast<int>(settings.components));
            dataSize = image.size.x * image.size.y * static_cast<int>(settings.components) * sizeof(uint16);
            break;
        }
        case channel_format::float_hdr:
        {
            imageData = stbi_loadf_from_memory(data.data(), data.size(), &image.size.x, &image.size.y, reinterpret_cast<int*>(&components), static_cast<int>(settings.components));
            dataSize = image.size.x * image.size.y * static_cast<int>(settings.components) * sizeof(float);
            break;
        }
        }

        image.format = settings.fileFormat;
        image.data = new byte[dataSize];
        image.dataSize = dataSize;
        image.components = settings.components;

        memmove(image.data, imageData, dataSize);
        stbi_image_free(imageData);

        return decay(Ok(image));
    }
}
