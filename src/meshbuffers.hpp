/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MESHBUFFERS_HPP
#define MESHBUFFERS_HPP

#include <memory>

#include <vulkan/vulkan.h>

class CommandPool;
class Device;
class Mesh;

/**
 * @brief A wrapper for Vulkan vertex and index buffer.
 */
class MeshBuffers {
  public:
    MeshBuffers(Device const &, CommandPool const &, Mesh const &);
    MeshBuffers(MeshBuffers const &) = delete;
    MeshBuffers &operator=(MeshBuffers const &) = delete;
    virtual ~MeshBuffers();
    std::shared_ptr<VkBuffer> GetVulkanIndexBuffer() const;
    std::shared_ptr<VkBuffer> GetVulkanVertexBuffer() const;
    uint32_t GetIndexCount() const;
    uint32_t GetVertexCount() const;
  
  private:
    int8_t CreateVulkanIndexBuffer(Device const &, CommandPool const &,
        Mesh const &);
    int8_t CreateVulkanVertexBuffer(Device const &, CommandPool const &,
        Mesh const &);

    std::shared_ptr<VkBuffer> m_vulkan_index_buffer;
    std::shared_ptr<VkBuffer> m_vulkan_vertex_buffer;
    std::shared_ptr<VkDeviceMemory> m_vulkan_index_buffer_memory;
    std::shared_ptr<VkDeviceMemory> m_vulkan_vertex_buffer_memory;
    uint32_t m_index_count;
    uint32_t m_vertex_count;
};

#endif
