#include <rendering/data/model.hpp>
#include <rendering/data/material.hpp>
#include <map>
#include <string>
#include <fstream>
namespace legion::rendering
{
    sparse_map<id_type, model> ModelCache::m_models;
    async::rw_spinlock ModelCache::m_modelLock;

    async::rw_spinlock ModelCache::m_modelNameLock;
    std::unordered_map<id_type, std::string> ModelCache::m_modelNames;
    bool model_handle::is_buffered() const
    {
        return ModelCache::get_model(id).buffered;
    }

    void model_handle::buffer_data(const buffer& matrixBuffer) const
    {
        ModelCache::buffer_model(id, matrixBuffer);
    }

    void model_handle::overwrite_buffer(buffer& newBuffer, uint bufferID, bool perInstance) const
    {
        ModelCache::overwrite_buffer(id, newBuffer, bufferID, perInstance);
    }

    mesh_handle model_handle::get_mesh() const
    {
        return ModelCache::get_mesh(id);
    }

    const model& model_handle::get_model() const
    {
        return ModelCache::get_model(id);
    }

    const model& ModelCache::get_model(id_type id)
    {
        async::readonly_guard guard(m_modelLock);
        return m_models[id];
    }

    std::string ModelCache::get_model_name(id_type id)
    {
        async::readonly_guard guard(m_modelNameLock);
        return m_modelNames[id];
    }

    void ModelCache::overwrite_buffer(id_type id, buffer& newBuffer, uint bufferID, bool perInstance)
    {
        //OPTICK_EVENT();
        if (id == invalid_id)
            return;
        //get mesh handle
        auto mesh_handle = MeshCache::get_handle(id);
        if (!mesh_handle)
            return;
        //get mesh and lock
        auto [lock, mesh] = mesh_handle.get();
        async::readonly_multiguard guard(m_modelLock, lock);
        auto& model = m_models[id];
        if (bufferID == SV_COLOR)
        {
            model.vertexArray.setAttribPointer(newBuffer, SV_COLOR, 4, GL_FLOAT, false, 0, 0);
            model.vertexArray.setAttribDivisor(SV_COLOR, perInstance);
        }
    }

    void ModelCache::buffer_model(id_type id, const buffer& matrixBuffer)
    {
        if (id == invalid_id)
            return;

        auto mesh_handle = MeshCache::get_handle(id);
        if (!mesh_handle)
            return;
        auto [lock, mesh] = mesh_handle.get();

        async::readonly_multiguard guard(m_modelLock, lock);
        model& model = m_models[id];

        model.vertexArray = vertexarray::generate();
        model.indexBuffer = buffer(GL_ELEMENT_ARRAY_BUFFER, mesh.indices, GL_STATIC_DRAW);

        model.vertexBuffer = buffer(GL_ARRAY_BUFFER, mesh.vertices, GL_STATIC_DRAW);
        model.vertexArray.setAttribPointer(model.vertexBuffer, SV_POSITION, 3, GL_FLOAT, false, 0, 0);

        model.colorBuffer = buffer(GL_ARRAY_BUFFER, mesh.colors, GL_STATIC_DRAW);
        model.vertexArray.setAttribPointer(model.colorBuffer, SV_COLOR, 4, GL_FLOAT, false, 0, 0);

        model.normalBuffer = buffer(GL_ARRAY_BUFFER, mesh.normals, GL_STATIC_DRAW);
        model.vertexArray.setAttribPointer(model.normalBuffer, SV_NORMAL, 3, GL_FLOAT, false, 0, 0);

        model.tangentBuffer = buffer(GL_ARRAY_BUFFER, mesh.tangents, GL_STATIC_DRAW);
        model.vertexArray.setAttribPointer(model.tangentBuffer, SV_TANGENT, 3, GL_FLOAT, false, 0, 0);

        model.uvBuffer = buffer(GL_ARRAY_BUFFER, mesh.uvs, GL_STATIC_DRAW);
        model.vertexArray.setAttribPointer(model.uvBuffer, SV_TEXCOORD0, 2, GL_FLOAT, false, 0, 0);

        model.vertexArray.setAttribPointer(matrixBuffer, SV_MODELMATRIX + 0, 4, GL_FLOAT, false, sizeof(math::mat4), 0 * sizeof(math::mat4::col_type));
        model.vertexArray.setAttribPointer(matrixBuffer, SV_MODELMATRIX + 1, 4, GL_FLOAT, false, sizeof(math::mat4), 1 * sizeof(math::mat4::col_type));
        model.vertexArray.setAttribPointer(matrixBuffer, SV_MODELMATRIX + 2, 4, GL_FLOAT, false, sizeof(math::mat4), 2 * sizeof(math::mat4::col_type));
        model.vertexArray.setAttribPointer(matrixBuffer, SV_MODELMATRIX + 3, 4, GL_FLOAT, false, sizeof(math::mat4), 3 * sizeof(math::mat4::col_type));

        model.vertexArray.setAttribDivisor(SV_MODELMATRIX + 0, 1);
        model.vertexArray.setAttribDivisor(SV_MODELMATRIX + 1, 1);
        model.vertexArray.setAttribDivisor(SV_MODELMATRIX + 2, 1);
        model.vertexArray.setAttribDivisor(SV_MODELMATRIX + 3, 1);

        model.buffered = true;
    }

    model_handle ModelCache::create_model(const std::string& name, const fs::view& file, mesh_import_settings settings)
    {
        id_type id = nameHash(name);

        {// Check if the model already exists.
            async::readonly_guard guard(m_modelLock);
            if (m_models.contains(id))
                return { id };
        }

        // Check if the file is valid to load.
        if (!file.is_valid() || !file.file_info().is_file)
            return invalid_model_handle;
        // Load the mesh if it wasn't already. (It's called MeshCache for a reason.)

        model model{};
        std::string meshName;

        if (settings.contextFolder.get_virtual_path() == "")
        {
            settings.contextFolder = file.parent();
        }

        auto handle = MeshCache::create_mesh(name, file, settings);
        if (handle == invalid_mesh_handle)
        {
            log::error("Failed to load model {}", name);
            return invalid_model_handle;
        }

        // Copy the sub-mesh data.
        auto [lock, data] = handle.get();
        async::readonly_guard guard(lock);
        meshName = data.fileName;

        for (auto& submeshData : data.submeshes)
            model.submeshes.push_back(submeshData);

        // The model still needs to be buffered on the rendering thread.
        model.buffered = false;

        { // Insert the model into the model list.
            async::readwrite_guard guard(m_modelLock);
            m_models.insert(id, model);
        }

        {
            async::readwrite_guard guard(m_modelNameLock);
            m_modelNames[id] = name;
        }

        log::debug("Created model {} with mesh: {}", name, meshName);

        return { id };
    }

    model_handle ModelCache::create_model(const std::string& name, const fs::view& file, std::vector<material_handle>& materials, mesh_import_settings settings)
    {
        id_type id = nameHash(name);

        {// Check if the model already exists.
            async::readonly_guard guard(m_modelLock);
            if (m_models.contains(id))
                return { id };
        }

        // Check if the file is valid to load.
        if (!file.is_valid() || !file.file_info().is_file)
            return invalid_model_handle;
        // Load the mesh if it wasn't already. (It's called MeshCache for a reason.)

        model model{};
        std::string meshName;

        material_list matList;
        material_list* loadedMaterials;
        if (settings.materials)
            loadedMaterials = settings.materials;
        else
        {
            loadedMaterials = &matList;
            settings.materials = &matList;
        }

        if (settings.contextFolder.get_virtual_path() == "")
        {
            settings.contextFolder = file.parent();
        }

        auto handle = MeshCache::create_mesh(name, file, settings);
        if (handle == invalid_mesh_handle)
        {
            log::error("Failed to load model {}", name);
            return invalid_model_handle;
        }

        if (loadedMaterials && loadedMaterials->size() > 0)
        {
            for (auto& mat : *loadedMaterials)
            {
                static auto defaultLitShader = ShaderCache::create_shader("default lit", fs::view("engine://shaders/default_lit.shs"));

                auto material = MaterialCache::create_material(name + "/" + mat.name, defaultLitShader);

                if (mat.doubleSided)
                    material.set_variant("double_sided");

                material.set_param("alphaCutoff", mat.alphaCutoff);

                if (mat.albedoMap)
                {
                    material.set_param("useAlbedoTex", true);
                    material.set_param("albedoTex", TextureCache::create_texture_from_image(mat.albedoMap));
                }
                else
                {
                    material.set_param("useAlbedoTex", false);
                    material.set_param("albedoColor", mat.albedoValue);
                }

                if (mat.metallicRoughnessMap)
                {
                    material.set_param("useMetallicRoughness", true);
                    material.set_param("metallicRoughness", TextureCache::create_texture_from_image(mat.metallicRoughnessMap));
                }
                else
                {
                    material.set_param("useMetallicRoughness", false);

                    if (mat.metallicMap)
                    {
                        material.set_param("useMetallicTex", true);
                        material.set_param("metallicTex", TextureCache::create_texture_from_image(mat.metallicMap));
                    }
                    else
                    {
                        material.set_param("useMetallicTex", false);
                        material.set_param("metallicValue", mat.metallicValue);
                    }

                    if (mat.roughnessMap)
                    {
                        material.set_param("useRoughnessTex", true);
                        material.set_param("roughnessTex", TextureCache::create_texture_from_image(mat.roughnessMap));
                    }
                    else
                    {
                        material.set_param("useRoughnessTex", false);
                        material.set_param("roughnessValue", mat.roughnessValue);
                    }
                }

                if (mat.emissiveMap)
                {
                    material.set_param("useEmissiveTex", true);
                    material.set_param("emissiveTex", TextureCache::create_texture_from_image(mat.emissiveMap));
                }
                else
                {
                    material.set_param("useEmissiveTex", false);
                    material.set_param("emissiveColor", mat.emissiveValue);
                }

                if (mat.normalMap)
                {
                    material.set_param("useNormal", true);
                    material.set_param("normalTex", TextureCache::create_texture_from_image(mat.normalMap));
                }
                else
                {
                    material.set_param("useNormal", false);
                }

                if (mat.aoMap)
                {
                    material.set_param("useAmbientOcclusion", true);
                    material.set_param("ambientOcclusionTex", TextureCache::create_texture_from_image(mat.aoMap));
                }
                else
                {
                    material.set_param("useAmbientOcclusion", false);
                }

                if (mat.heightMap)
                {
                    material.set_param("useHeight", true);
                    material.set_param("heightTex", TextureCache::create_texture_from_image(mat.heightMap));
                }
                else
                {
                    material.set_param("useHeight", false);
                }

                materials.push_back(material);
                log::debug("Loaded embedded material {}/{}", name, mat.name);
            }
        }

        // Copy the sub-mesh data.
        auto [lock, data] = handle.get();
        async::readonly_guard guard(lock);
        meshName = data.fileName;

        for (auto& submeshData : data.submeshes)
            model.submeshes.push_back(submeshData);

        // The model still needs to be buffered on the rendering thread.
        model.buffered = false;

        { // Insert the model into the model list.
            async::readwrite_guard guard(m_modelLock);
            m_models.insert(id, model);
        }

        {
            async::readwrite_guard guard(m_modelNameLock);
            m_modelNames[id] = name;
        }

        log::debug("Created model {} with mesh: {}", name, meshName);

        return { id };
    }

    model_handle ModelCache::create_model(const std::string& name)
    {
        id_type id = nameHash(name);

        {// Check if the model already exists.
            async::readonly_guard guard(m_modelLock);
            if (m_models.contains(id))
                return { id };
        }

        model model{};
        std::string meshName;

        {// Load the mesh if it wasn't already. (It's called MeshCache for a reason.)
            auto handle = MeshCache::get_handle(name);
            if (handle == invalid_mesh_handle)
            {
                log::error("Failed to load model {}", name);
                return invalid_model_handle;
            }

            // Copy the sub-mesh data.
            auto [lock, data] = handle.get();
            async::readonly_guard guard(lock);
            meshName = data.fileName;

            for (auto& submeshData : data.submeshes)
                model.submeshes.push_back(submeshData);
        }

        // The model still needs to be buffered on the rendering thread.
        model.buffered = false;

        { // Insert the model into the model list.
            async::readwrite_guard guard(m_modelLock);
            m_models.insert(id, model);
        }

        {
            async::readwrite_guard guard(m_modelNameLock);
            m_modelNames[id] = name;
        }

        log::trace("Created model {} with mesh: {}", name, meshName);

        return { id };
    }

    model_handle ModelCache::create_model(const std::string& name, id_type meshId)
    {
        id_type id = nameHash(name);

        {// Check if the model already exists.
            async::readonly_guard guard(m_modelLock);
            if (m_models.contains(id))
                return { id };
        }

        model model{};
        std::string meshName;

        {// Load the mesh if it wasn't already. (It's called MeshCache for a reason.)
            auto handle = MeshCache::get_handle(meshId);
            if (handle == invalid_mesh_handle)
            {
                log::error("Failed to load model {}", name);
                return invalid_model_handle;
            }

            // Copy the sub-mesh data.
            auto [lock, data] = handle.get();
            async::readonly_guard guard(lock);
            meshName = data.fileName;

            for (auto& submeshData : data.submeshes)
                model.submeshes.push_back(submeshData);
        }

        // The model still needs to be buffered on the rendering thread.
        model.buffered = false;

        { // Insert the model into the model list.
            async::readwrite_guard guard(m_modelLock);
            m_models.insert(id, model);
        }

        {
            async::readwrite_guard guard(m_modelNameLock);
            m_modelNames[id] = name;
        }

        log::trace("Created model {} with mesh: {}", name, meshName);

        return { id };
    }

    model_handle ModelCache::create_model(id_type id)
    {
        {// Check if the model already exists.
            async::readonly_guard guard(m_modelLock);
            if (m_models.contains(id))
                return { id };
        }

        model model{};
        std::string meshName;

        {// Load the mesh if it wasn't already. (It's called MeshCache for a reason.)
            auto handle = MeshCache::get_handle(id);
            if (handle == invalid_mesh_handle)
            {
                log::error("Failed to load model {}", id);
                return invalid_model_handle;
            }

            // Copy the sub-mesh data.
            auto [lock, data] = handle.get();
            async::readonly_guard guard(lock);
            meshName = data.fileName;

            for (auto& submeshData : data.submeshes)
                model.submeshes.push_back(submeshData);
        }

        // The model still needs to be buffered on the rendering thread.
        model.buffered = false;

        { // Insert the model into the model list.
            async::readwrite_guard guard(m_modelLock);
            m_models.insert(id, model);
        }

        {
            async::readwrite_guard guard(m_modelNameLock);
            m_modelNames[id] = std::to_string(id);
        }

        log::trace("Created model {} with mesh: {}", id, meshName);

        return { id };
    }

    model_handle ModelCache::create_model(const std::string& name, mesh_handle mesh)
    {
        id_type id = nameHash(name);

        {// Check if the model already exists.
            async::readonly_guard guard(m_modelLock);
            if (m_models.contains(id))
                return { id };
        }

        model model{};
        std::string meshName;

        {// Load the mesh if it wasn't already. (It's called MeshCache for a reason.)
            if (mesh == invalid_mesh_handle)
            {
                log::error("Failed to load model {}", name);
                return invalid_model_handle;
            }

            // Copy the sub-mesh data.
            auto [lock, data] = mesh.get();
            async::readonly_guard guard(lock);
            meshName = data.fileName;

            for (auto& submeshData : data.submeshes)
                model.submeshes.push_back(submeshData);
        }

        // The model still needs to be buffered on the rendering thread.
        model.buffered = false;

        { // Insert the model into the model list.
            async::readwrite_guard guard(m_modelLock);
            m_models.insert(id, model);
        }

        {
            async::readwrite_guard guard(m_modelNameLock);
            m_modelNames[id] = name;
        }

        log::trace("Created model {} with mesh: {}", name, meshName);

        return { id };
    }

    model_handle ModelCache::create_model(mesh_handle mesh)
    {
        id_type id = mesh.id;

        {// Check if the model already exists.
            async::readonly_guard guard(m_modelLock);
            if (m_models.contains(id))
                return { id };
        }

        model model{};
        std::string meshName;

        {// Load the mesh if it wasn't already. (It's called MeshCache for a reason.)
            if (mesh == invalid_mesh_handle)
            {
                log::error("Failed to load model {}", id);
                return invalid_model_handle;
            }

            // Copy the sub-mesh data.
            auto [lock, data] = mesh.get();
            async::readonly_guard guard(lock);
            meshName = data.fileName;

            for (auto& submeshData : data.submeshes)
                model.submeshes.push_back(submeshData);
        }

        // The model still needs to be buffered on the rendering thread.
        model.buffered = false;

        { // Insert the model into the model list.
            async::readwrite_guard guard(m_modelLock);
            m_models.insert(id, model);
        }

        {
            auto [lock, rawmesh] = mesh.get();
            async::mixed_multiguard guard(m_modelNameLock, async::lock_state_read, lock, async::lock_state_write);
            m_modelNames[id] = rawmesh.fileName;
        }

        log::trace("Created model {} with mesh: {}", id, meshName);

        return { id };
    }

    model_handle ModelCache::get_handle(const std::string& name)
    {
        id_type id = nameHash(name);
        async::readonly_guard guard(m_modelLock);
        if (m_models.contains(id))
            return { id };
        return invalid_model_handle;
    }

    model_handle ModelCache::get_handle(id_type id)
    {
        async::readonly_guard guard(m_modelLock);
        if (m_models.contains(id))
            return { id };
        return invalid_model_handle;
    }

    mesh_handle ModelCache::get_mesh(id_type id)
    {
        return MeshCache::get_handle(id);
    }

    mesh_handle ModelCache::get_mesh(const std::string& name)
    {
        id_type id = nameHash(name);
        return MeshCache::get_handle(id);
    }
}
