#pragma once
#include <core/types/primitives.hpp>
#include <core/math/math.hpp>
#include <core/filesystem/resource.hpp>
#include <core/filesystem/view.hpp>
#include <core/async/rw_spinlock.hpp>
#include <core/data/image.hpp>

#include <utility>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>

/**
 * @file mesh.hpp
 */

namespace legion::core
{
    /**@class sub_mesh
     * @brief Encapsulation of a sub-mesh with the offsets and sizes of the sub-mesh within the main mesh data.
     */
    struct sub_mesh
    {
        std::string name;
        size_type indexCount;
        size_type indexOffset;
    };

    /**@class mesh
     * @brief Raw mesh representation.
     */
    struct mesh
    {
        std::string fileName;
        std::vector<math::vec3> vertices;
        std::vector<math::color> colors;
        std::vector<math::vec3> normals;
        std::vector<math::vec2> uvs;
        std::vector<math::vec3> tangents;
        std::vector<uint> indices;

        std::vector<sub_mesh> submeshes;

        /**@brief Standard to resource conversion.
         */
        static void to_resource(filesystem::basic_resource* resource, const mesh& value);

        /**@brief Standard from resource conversion.
         */
        static void from_resource(mesh* value, const filesystem::basic_resource& resource);

        /**@brief Calculate the tangents from the triangles, vertices and normals of a certain mesh.
         */
        static void calculate_tangents(mesh* data);
    };

    /**@class mesh_handle
     * @brief Save to pass around handle to a raw mesh in the mesh cache.
     */
    struct mesh_handle
    {
        id_type id;

        /**@brief Get the mesh and the attached lock.
         */
        std::pair<async::rw_spinlock&, mesh&> get();

        bool operator==(const mesh_handle& other) const { return id == other.id; }
        operator id_type() { return id; }
    };

    /**@brief Default invalid mesh handle.
     */
    constexpr mesh_handle invalid_mesh_handle{ invalid_id };

    struct material_data
    {
        std::string name;

        bool opaque;            
        float alphaCutoff;                
        bool doubleSided;                 

        math::color albedoValue;
        image_handle albedoMap;
        float metallicValue;
        image_handle metallicMap;
        float roughnessValue;
        image_handle roughnessMap;
        image_handle metallicRoughnessMap;
        math::color emissiveValue;
        image_handle emissiveMap;

        image_handle normalMap;
        image_handle aoMap;
        image_handle heightMap;
    };

    using material_list = std::vector<material_data>;

    /**@class mesh_import_settings
     * @brief Data structure to parameterize the mesh import process.
     */
    struct mesh_import_settings
    {
        material_list* materials = nullptr;
        bool triangulate = true;
        bool vertex_color = false;
        filesystem::view contextFolder = filesystem::view(std::string_view(""));
    };

    /**@brief Default mesh import settings.
     */
    const mesh_import_settings default_mesh_settings{ nullptr, true, false, filesystem::view(std::string_view("")) };

    /**@class MeshCache
     * @brief Data cache for loading, storing and managing raw meshes.
     */
    class MeshCache
    {
        friend struct mesh_handle;
    private:
        static std::unordered_map<id_type, std::unique_ptr<std::pair<async::rw_spinlock, mesh>>> m_meshes;
        static async::rw_spinlock m_meshesLock;

    public:
        /**@brief Create a new mesh and load it from a file if a mesh with the same name doesn't exist yet.
         * @param name Identifying name for the mesh.
         * @param file File to load from.
         * @param settings Settings to pass on to the import pipeline.
         * @return mesh_handle A valid handle to the newly created mesh if it succeeds, invalid_mesh_handle if it fails.
         */
        static mesh_handle create_mesh(const std::string& name, const filesystem::view& file, mesh_import_settings settings = default_mesh_settings);

        static mesh_handle create_mesh(const std::string& name, const mesh& mesh);

        /**@brief Copy a mesh with a certain name to a new name. Will overwrite the destination if that mesh already existed.
         * @param name Source name
         * @param newName Destination name
         * @return mesh_handle A valid handle to the copy.
         */
        static mesh_handle copy_mesh(const std::string& name, const std::string& newName);

        /**@brief Copy a mesh with a certain name to a new name. Will overwrite the destination if that mesh already existed.
         * @param id Source name hash
         * @param newName Destination name
         * @return mesh_handle A valid handle to the copy.
         */
        static mesh_handle copy_mesh(id_type id, const std::string& newName);

        /**@brief Returns a handle to a mesh with a certain name. Will return invalid_mesh_handle if the requested mesh doesn't exist.
         */
        static mesh_handle get_handle(const std::string& name);

        /**@brief Returns a handle to a mesh with a certain name. Will return invalid_mesh_handle if the requested mesh doesn't exist.
         * @param id Name hash
         */
        static mesh_handle get_handle(id_type id);
    };
}
