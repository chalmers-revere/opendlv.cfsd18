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

#include <cstring>

#include "commandpool.hpp"
#include "device.hpp"
#include "mesh.hpp"
#include "meshbuffers.hpp"
#include "vertex.hpp"

MeshBuffers::MeshBuffers(Device const &a_device, 
    CommandPool const &a_command_pool, Mesh const &a_mesh): 
  m_vulkan_index_buffer(),
  m_vulkan_vertex_buffer(),
  m_vulkan_index_buffer_memory(),
  m_vulkan_vertex_buffer_memory(),
  m_index_count(),
  m_vertex_count()
{
  CreateVulkanIndexBuffer(a_device, a_command_pool, a_mesh); 
  CreateVulkanVertexBuffer(a_device, a_command_pool, a_mesh); 
}

MeshBuffers::~MeshBuffers()
{
}

int8_t MeshBuffers::CreateVulkanIndexBuffer(Device const &a_device, 
    CommandPool const &a_command_pool, Mesh const &a_mesh)
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto indices = a_mesh.GetIndices();
  m_index_count = indices->size();

  VkDeviceSize buffer_size = sizeof(indices->at(0)) * m_index_count;

  VkBufferCreateInfo staging_buffer_info = {};
  staging_buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  staging_buffer_info.size = buffer_size;
  staging_buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  staging_buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  
  std::shared_ptr<VkBuffer> staging_buffer(new VkBuffer, 
      [vulkan_device](VkBuffer *a_staging_buffer) {
        vkDestroyBuffer(*vulkan_device, *a_staging_buffer, nullptr);
        std::cout << "DEBUG: Index staging buffer deleted." << std::endl;
      });

  int32_t res = vkCreateBuffer(*vulkan_device, &staging_buffer_info, nullptr, 
      &(*staging_buffer));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create index staging buffer." << std::endl;
    return -1;
  }

  VkMemoryPropertyFlags staging_memory_properties = 
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

  VkMemoryRequirements staging_memory_requirements;
  vkGetBufferMemoryRequirements(*vulkan_device, *staging_buffer, 
      &staging_memory_requirements);

  bool has_staging_memory_type = a_device.HasMemoryType(
      staging_memory_requirements, staging_memory_properties);
  if (!has_staging_memory_type) {
    std::cerr << "Failed to find suitable memory type for staging buffer." 
      << std::endl;
    return -1;
  }

  uint32_t staging_memory_type_index = a_device.FindMemoryTypeIndex(
      staging_memory_requirements, staging_memory_properties);

  VkMemoryAllocateInfo staging_allocation_info = {};
  staging_allocation_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  staging_allocation_info.allocationSize = staging_memory_requirements.size;
  staging_allocation_info.memoryTypeIndex = staging_memory_type_index;

  std::shared_ptr<VkDeviceMemory> staging_buffer_memory(new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_staging_buffer_memory) {
        vkFreeMemory(*vulkan_device, *a_staging_buffer_memory, nullptr);
        std::cout << "DEBUG: Index staging buffer memory freed." << std::endl;
      });

  res = vkAllocateMemory(*vulkan_device, &staging_allocation_info, nullptr, 
      &(*staging_buffer_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate index staging buffer memory." 
      << std::endl;
    return -1;
  }

  vkBindBufferMemory(*vulkan_device, *staging_buffer, *staging_buffer_memory, 
      0);
  
  // TODO: Free 'data' later??
  void *data;
  vkMapMemory(*vulkan_device, *staging_buffer_memory, 0, buffer_size, 0, &data);
  memcpy(data, indices->data(), static_cast<size_t>(buffer_size));
  vkUnmapMemory(*vulkan_device, *staging_buffer_memory);


  VkBufferCreateInfo buffer_info = {};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = buffer_size;
  buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT
    | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  m_vulkan_index_buffer.reset(new VkBuffer, 
      [vulkan_device](VkBuffer *a_index_buffer) {
        vkDestroyBuffer(*vulkan_device, *a_index_buffer, nullptr);
        std::cout << "DEBUG: Index buffer deleted." << std::endl;
      });

  res = vkCreateBuffer(*vulkan_device, &buffer_info, nullptr, 
      &(*m_vulkan_index_buffer));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create index buffer." << std::endl;
    return -1;
  }

  VkMemoryPropertyFlags index_memory_properties = 
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

  VkMemoryRequirements index_memory_requirements;
  vkGetBufferMemoryRequirements(*vulkan_device, *m_vulkan_index_buffer, 
      &index_memory_requirements);

  bool has_index_memory_type = a_device.HasMemoryType(
      index_memory_requirements, index_memory_properties);
  if (!has_index_memory_type) {
    std::cerr << "Failed to find suitable memory type for index buffer." 
      << std::endl;
    return -1;
  }

  uint32_t index_memory_type_index = a_device.FindMemoryTypeIndex(
      index_memory_requirements, index_memory_properties);

  VkMemoryAllocateInfo allocation_info = {};
  allocation_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocation_info.allocationSize = index_memory_requirements.size;
  allocation_info.memoryTypeIndex = index_memory_type_index;

  m_vulkan_index_buffer_memory.reset(new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_index_buffer_memory) {
        vkFreeMemory(*vulkan_device, *a_index_buffer_memory, nullptr);
        std::cout << "DEBUG: Index buffer memory freed." << std::endl;
      });

  res = vkAllocateMemory(*vulkan_device, &allocation_info, nullptr, 
      &(*m_vulkan_index_buffer_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate index buffer memory." 
      << std::endl;
    return -1;
  }

  vkBindBufferMemory(*vulkan_device, *m_vulkan_index_buffer, 
      *m_vulkan_index_buffer_memory, 0);


  VkCommandBuffer command_buffer = 
    a_command_pool.BeginSingleTimeCommands(a_device);

  VkBufferCopy copy_region = {};
  copy_region.size = buffer_size;
  vkCmdCopyBuffer(command_buffer, *staging_buffer, *m_vulkan_index_buffer, 1, 
      &copy_region);

  a_command_pool.EndSingleTimeCommands(a_device, command_buffer);

  return 0;
}

int8_t MeshBuffers::CreateVulkanVertexBuffer(Device const &a_device,
    CommandPool const &a_command_pool, Mesh const &a_mesh)
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vertices = a_mesh.GetVertices();
  m_vertex_count = vertices->size();

  VkDeviceSize buffer_size = sizeof(vertices->at(0)) * m_vertex_count;

  VkBufferCreateInfo staging_buffer_info = {};
  staging_buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  staging_buffer_info.size = buffer_size;
  staging_buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  staging_buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  
  std::shared_ptr<VkBuffer> staging_buffer(new VkBuffer, 
      [vulkan_device](VkBuffer *a_staging_buffer) {
        vkDestroyBuffer(*vulkan_device, *a_staging_buffer, nullptr);
        std::cout << "DEBUG: Vertex staging buffer deleted." << std::endl;
      });

  int32_t res = vkCreateBuffer(*vulkan_device, &staging_buffer_info, nullptr, 
      &(*staging_buffer));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create vertex staging buffer." << std::endl;
    return -1;
  }

  VkMemoryPropertyFlags staging_memory_properties = 
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

  VkMemoryRequirements staging_memory_requirements;
  vkGetBufferMemoryRequirements(*vulkan_device, *staging_buffer, 
      &staging_memory_requirements);

  bool has_staging_memory_type = a_device.HasMemoryType(
      staging_memory_requirements, staging_memory_properties);
  if (!has_staging_memory_type) {
    std::cerr << "Failed to find suitable memory type for staging buffer." 
      << std::endl;
    return -1;
  }

  uint32_t staging_memory_type_index = a_device.FindMemoryTypeIndex(
      staging_memory_requirements, staging_memory_properties);

  VkMemoryAllocateInfo staging_allocation_info = {};
  staging_allocation_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  staging_allocation_info.allocationSize = staging_memory_requirements.size;
  staging_allocation_info.memoryTypeIndex = staging_memory_type_index;

  std::shared_ptr<VkDeviceMemory> staging_buffer_memory(new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_staging_buffer_memory) {
        vkFreeMemory(*vulkan_device, *a_staging_buffer_memory, nullptr);
        std::cout << "DEBUG: Vertex staging buffer memory freed." << std::endl;
      });

  res = vkAllocateMemory(*vulkan_device, &staging_allocation_info, nullptr, 
      &(*staging_buffer_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate vertex staging buffer memory." 
      << std::endl;
    return -1;
  }

  vkBindBufferMemory(*vulkan_device, *staging_buffer, *staging_buffer_memory, 
      0);
  
  // TODO: Free 'data' later??
  void *data;
  vkMapMemory(*vulkan_device, *staging_buffer_memory, 0, buffer_size, 0, &data);
  memcpy(data, vertices->data(), static_cast<size_t>(buffer_size));
  vkUnmapMemory(*vulkan_device, *staging_buffer_memory);


  VkBufferCreateInfo buffer_info = {};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = buffer_size;
  buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT
    | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  m_vulkan_vertex_buffer.reset(new VkBuffer, 
      [vulkan_device](VkBuffer *a_vertex_buffer) {
        vkDestroyBuffer(*vulkan_device, *a_vertex_buffer, nullptr);
        std::cout << "DEBUG: Vertex buffer deleted." << std::endl;
      });

  res = vkCreateBuffer(*vulkan_device, &buffer_info, nullptr, 
      &(*m_vulkan_vertex_buffer));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create vertex buffer." << std::endl;
    return -1;
  }

  VkMemoryPropertyFlags vertex_memory_properties = 
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

  VkMemoryRequirements vertex_memory_requirements;
  vkGetBufferMemoryRequirements(*vulkan_device, *m_vulkan_vertex_buffer, 
      &vertex_memory_requirements);

  bool has_vertex_memory_type = a_device.HasMemoryType(
      vertex_memory_requirements, vertex_memory_properties);
  if (!has_vertex_memory_type) {
    std::cerr << "Failed to find suitable memory type for vertex buffer." 
      << std::endl;
    return -1;
  }

  uint32_t vertex_memory_type_index = a_device.FindMemoryTypeIndex(
      vertex_memory_requirements, vertex_memory_properties);

  VkMemoryAllocateInfo allocation_info = {};
  allocation_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocation_info.allocationSize = vertex_memory_requirements.size;
  allocation_info.memoryTypeIndex = vertex_memory_type_index;

  m_vulkan_vertex_buffer_memory.reset(new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_vertex_buffer_memory) {
        vkFreeMemory(*vulkan_device, *a_vertex_buffer_memory, nullptr);
        std::cout << "DEBUG: Vertex buffer memory freed." << std::endl;
      });

  res = vkAllocateMemory(*vulkan_device, &allocation_info, nullptr, 
      &(*m_vulkan_vertex_buffer_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate vertex buffer memory." 
      << std::endl;
    return -1;
  }

  vkBindBufferMemory(*vulkan_device, *m_vulkan_vertex_buffer, 
      *m_vulkan_vertex_buffer_memory, 0);


  VkCommandBuffer command_buffer = 
    a_command_pool.BeginSingleTimeCommands(a_device);

  VkBufferCopy copy_region = {};
  copy_region.size = buffer_size;
  vkCmdCopyBuffer(command_buffer, *staging_buffer, *m_vulkan_vertex_buffer, 1, 
      &copy_region);

  a_command_pool.EndSingleTimeCommands(a_device, command_buffer);

  return 0;
}

std::shared_ptr<VkBuffer> MeshBuffers::GetVulkanIndexBuffer() const
{
  return m_vulkan_index_buffer;
}

std::shared_ptr<VkBuffer> MeshBuffers::GetVulkanVertexBuffer() const
{
  return m_vulkan_vertex_buffer;
}

uint32_t MeshBuffers::GetIndexCount() const
{
  return m_index_count;
}

uint32_t MeshBuffers::GetVertexCount() const
{
  return m_vertex_count;
}
