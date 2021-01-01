#include <rendering/data/model.hpp>

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

        /*glGenVertexArrays(1, &model.vertexArrayId);
        glBindVertexArray(model.vertexArrayId);

        glGenBuffers(1, &model.indexBufferId);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model.indexBufferId);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(uint), mesh.indices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        glGenBuffers(1, &model.vertexBufferId);
        glBindBuffer(GL_ARRAY_BUFFER, model.vertexBufferId);
        glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(math::vec3), mesh.vertices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(SV_POSITION);
        glVertexAttribPointer(SV_POSITION, 3, GL_FLOAT, GL_FALSE, 0, 0);

        glGenBuffers(1, &model.colorBufferId);
        glBindBuffer(GL_ARRAY_BUFFER, model.colorBufferId);
        glBufferData(GL_ARRAY_BUFFER, mesh.colors.size() * sizeof(math::color), mesh.colors.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(SV_COLOR);
        glVertexAttribPointer(SV_COLOR, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glGenBuffers(1, &model.normalBufferId);
        glBindBuffer(GL_ARRAY_BUFFER, model.normalBufferId);
        glBufferData(GL_ARRAY_BUFFER, mesh.normals.size() * sizeof(math::vec3), mesh.normals.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(SV_NORMAL);
        glVertexAttribPointer(SV_NORMAL, 3, GL_FLOAT, GL_FALSE, 0, 0);

        glGenBuffers(1, &model.tangentBufferId);
        glBindBuffer(GL_ARRAY_BUFFER, model.tangentBufferId);
        glBufferData(GL_ARRAY_BUFFER, mesh.tangents.size() * sizeof(math::vec3), mesh.tangents.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(SV_TANGENT);
        glVertexAttribPointer(SV_TANGENT, 3, GL_FLOAT, GL_FALSE, 0, 0);

        glGenBuffers(1, &model.uvBufferId);
        glBindBuffer(GL_ARRAY_BUFFER, model.uvBufferId);
        glBufferData(GL_ARRAY_BUFFER, mesh.uvs.size() * sizeof(math::vec2), mesh.uvs.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(SV_TEXCOORD0);
        glVertexAttribPointer(SV_TEXCOORD0, 2, GL_FLOAT, GL_FALSE, 0, 0);

        glBindBuffer(GL_ARRAY_BUFFER, matrixBuffer);
        glEnableVertexAttribArray(SV_MODELMATRIX + 0);
        glEnableVertexAttribArray(SV_MODELMATRIX + 1);
        glEnableVertexAttribArray(SV_MODELMATRIX + 2);
        glEnableVertexAttribArray(SV_MODELMATRIX + 3);

        glVertexAttribPointer(SV_MODELMATRIX + 0, 4, GL_FLOAT, GL_FALSE, sizeof(math::mat4), (GLvoid*)(0 * sizeof(math::mat4::col_type)));
        glVertexAttribPointer(SV_MODELMATRIX + 1, 4, GL_FLOAT, GL_FALSE, sizeof(math::mat4), (GLvoid*)(1 * sizeof(math::mat4::col_type)));
        glVertexAttribPointer(SV_MODELMATRIX + 2, 4, GL_FLOAT, GL_FALSE, sizeof(math::mat4), (GLvoid*)(2 * sizeof(math::mat4::col_type)));
        glVertexAttribPointer(SV_MODELMATRIX + 3, 4, GL_FLOAT, GL_FALSE, sizeof(math::mat4), (GLvoid*)(3 * sizeof(math::mat4::col_type)));

        glVertexAttribDivisor(SV_MODELMATRIX + 0, 1);
        glVertexAttribDivisor(SV_MODELMATRIX + 1, 1);
        glVertexAttribDivisor(SV_MODELMATRIX + 2, 1);
        glVertexAttribDivisor(SV_MODELMATRIX + 3, 1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);*/

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

        model model{};
        std::string meshName;

        {// Load the mesh if it wasn't already. (It's called MeshCache for a reason.)
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
