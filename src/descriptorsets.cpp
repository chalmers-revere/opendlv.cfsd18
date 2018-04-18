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

#include "descriptorsets.hpp"
#include "descriptorsetlayout.hpp"
#include "device.hpp"
#include "textureresources.hpp"
#include "texturesampler.hpp"
#include "uniformbuffer.hpp"
#include "uniformbufferobject.hpp"

DescriptorSets::DescriptorSets(Device const &a_device, 
    DescriptorSetLayout const &a_descriptor_set_layout,
    UniformBuffer const &a_uniform_buffer, 
    TextureResources const &a_texture_resources,
    TextureSampler const &a_texture_sampler): 
  m_vulkan_descriptor_pool(),
  m_vulkan_descriptor_set()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  CreateVulkanDescriptorPool(*vulkan_device); 

  auto vulkan_descriptor_set_layout = 
    a_descriptor_set_layout.GetVulkanDescriptorSetLayout();
  auto vulkan_uniform_buffer = a_uniform_buffer.GetVulkanUniformBuffer();
  auto vulkan_texture_image_view = 
    a_texture_resources.GetVulkanTextureImageView(); 
  auto vulkan_texture_sampler = a_texture_sampler.GetVulkanTextureSampler();

  UpdateVulkanDescriptorSet(*vulkan_device, *vulkan_descriptor_set_layout,
        *vulkan_uniform_buffer, *vulkan_texture_image_view, 
        *vulkan_texture_sampler);
}

DescriptorSets::~DescriptorSets()
{
}

int8_t DescriptorSets::CreateVulkanDescriptorPool(
    VkDevice const &a_vulkan_device)
{
  std::array<VkDescriptorPoolSize, 2> pool_sizes = {};
  pool_sizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  pool_sizes[0].descriptorCount = 1;
  pool_sizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  pool_sizes[1].descriptorCount = 1;

  VkDescriptorPoolCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  create_info.poolSizeCount = static_cast<uint32_t>(pool_sizes.size());
  create_info.pPoolSizes = pool_sizes.data();
  create_info.maxSets = 1;

  m_vulkan_descriptor_pool.reset(new VkDescriptorPool, 
      [a_vulkan_device](VkDescriptorPool *a_descriptor_pool) {
        vkDestroyDescriptorPool(a_vulkan_device, *a_descriptor_pool, nullptr);
        std::cout << "DEBUG: Descriptor pool deleted." << std::endl;
      });

  uint32_t res = vkCreateDescriptorPool(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_descriptor_pool));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create descriptor pool." << std::endl;
    return -1;
  }

  return 0;
}
    
int8_t DescriptorSets::UpdateVulkanDescriptorSet(
    VkDevice const &a_vulkan_device,
    VkDescriptorSetLayout const &a_vulkan_descriptor_set_layout,
    VkBuffer const &a_vulkan_uniform_buffer,
    VkImageView const &a_vulkan_texture_image_view,
    VkSampler const &a_vulkan_texture_sampler)
{
  VkDescriptorSetLayout layouts[] = {a_vulkan_descriptor_set_layout};

  VkDescriptorSetAllocateInfo allocate_info = {};
  allocate_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  allocate_info.descriptorPool = *m_vulkan_descriptor_pool;
  allocate_info.descriptorSetCount = 1;
  allocate_info.pSetLayouts = layouts;

  // Does not need to be freed (due to optional flag).
  m_vulkan_descriptor_set.reset(new VkDescriptorSet);

  uint32_t res = vkAllocateDescriptorSets(a_vulkan_device, &allocate_info, 
      &(*m_vulkan_descriptor_set));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create descriptor set." << std::endl;
    return -1;
  }

  VkDescriptorBufferInfo buffer_info = {};
  buffer_info.buffer = a_vulkan_uniform_buffer;
  buffer_info.offset = 0;
  buffer_info.range = sizeof(UniformBufferObject);

  VkDescriptorImageInfo image_info = {};
  image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  image_info.imageView = a_vulkan_texture_image_view;
  image_info.sampler = a_vulkan_texture_sampler;

  std::array<VkWriteDescriptorSet, 2> descriptor_writes = {};

  descriptor_writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  descriptor_writes[0].dstSet = *m_vulkan_descriptor_set;
  descriptor_writes[0].dstBinding = 0;
  descriptor_writes[0].dstArrayElement = 0;
  descriptor_writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  descriptor_writes[0].descriptorCount = 1;
  descriptor_writes[0].pBufferInfo = &buffer_info;

  descriptor_writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  descriptor_writes[1].dstSet = *m_vulkan_descriptor_set;
  descriptor_writes[1].dstBinding = 1;
  descriptor_writes[1].dstArrayElement = 0;
  descriptor_writes[1].descriptorType = 
    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  descriptor_writes[1].descriptorCount = 1;
  descriptor_writes[1].pImageInfo = &image_info;

  vkUpdateDescriptorSets(a_vulkan_device, 
      static_cast<uint32_t>(descriptor_writes.size()), descriptor_writes.data(),
      0, nullptr);

  return 0;
}

std::shared_ptr<VkDescriptorSet> DescriptorSets::GetVulkanDescriptorSet() const
{
  return m_vulkan_descriptor_set;
}
