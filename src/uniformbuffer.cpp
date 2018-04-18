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

#include "device.hpp"
#include "uniformbuffer.hpp"
#include "uniformbufferobject.hpp"

UniformBuffer::UniformBuffer(Device const &a_device): 
  m_vulkan_uniform_buffer(),
  m_vulkan_uniform_buffer_memory()
{
  CreateVulkanUniformBuffer(a_device); 
}

UniformBuffer::~UniformBuffer()
{
}

int8_t UniformBuffer::CreateVulkanUniformBuffer(Device const &a_device)
{
  auto vulkan_device = a_device.GetVulkanDevice();

  VkDeviceSize buffer_size = sizeof(UniformBufferObject);

  VkBufferCreateInfo buffer_info = {};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = buffer_size;
  buffer_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  m_vulkan_uniform_buffer.reset(new VkBuffer, 
      [vulkan_device](VkBuffer *a_uniform_buffer) {
        vkDestroyBuffer(*vulkan_device, *a_uniform_buffer, nullptr);
        std::cout << "DEBUG: Uniform buffer deleted." << std::endl;
      });

  uint32_t res = vkCreateBuffer(*vulkan_device, &buffer_info, nullptr, 
      &(*m_vulkan_uniform_buffer));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create uniform buffer." << std::endl;
    return -1;
  }

  VkMemoryPropertyFlags uniform_memory_properties = 
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

  VkMemoryRequirements uniform_memory_requirements;
  vkGetBufferMemoryRequirements(*vulkan_device, *m_vulkan_uniform_buffer, 
      &uniform_memory_requirements);

  bool has_uniform_memory_type = a_device.HasMemoryType(
      uniform_memory_requirements, uniform_memory_properties);
  if (!has_uniform_memory_type) {
    std::cerr << "Failed to find suitable memory type for uniform buffer." 
      << std::endl;
    return -1;
  }

  uint32_t uniform_memory_type_index = a_device.FindMemoryTypeIndex(
      uniform_memory_requirements, uniform_memory_properties);

  VkMemoryAllocateInfo allocation_info = {};
  allocation_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocation_info.allocationSize = uniform_memory_requirements.size;
  allocation_info.memoryTypeIndex = uniform_memory_type_index;

  m_vulkan_uniform_buffer_memory.reset(new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_uniform_buffer_memory) {
        vkFreeMemory(*vulkan_device, *a_uniform_buffer_memory, nullptr);
        std::cout << "DEBUG: Uniform buffer memory freed." << std::endl;
      });

  res = vkAllocateMemory(*vulkan_device, &allocation_info, nullptr, 
      &(*m_vulkan_uniform_buffer_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate uniform buffer memory." 
      << std::endl;
    return -1;
  }

  vkBindBufferMemory(*vulkan_device, *m_vulkan_uniform_buffer, 
      *m_vulkan_uniform_buffer_memory, 0);
  
  return 0;
}

std::shared_ptr<VkBuffer> UniformBuffer::GetVulkanUniformBuffer() const
{
  return m_vulkan_uniform_buffer;
}

void UniformBuffer::UpdateUniformBufferObject(Device const &a_device, 
    UniformBufferObject const &a_ubo)
{
  auto vulkan_device = a_device.GetVulkanDevice();

  void* data;
  vkMapMemory(*vulkan_device, *m_vulkan_uniform_buffer_memory, 0, sizeof(a_ubo),
      0, &data);
  memcpy(data, &a_ubo, sizeof(a_ubo));
  vkUnmapMemory(*vulkan_device, *m_vulkan_uniform_buffer_memory);
}
