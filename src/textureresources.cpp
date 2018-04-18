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

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "commandpool.hpp"
#include "device.hpp"
#include "textureresources.hpp"

TextureResources::TextureResources(Device const &a_device, 
    CommandPool const &a_command_pool, std::string const &a_texture_path): 
  m_vulkan_texture_image_memory(),
  m_vulkan_texture_image(),
  m_vulkan_texture_image_view()
{
  CreateVulkanTextureImage(a_device, a_command_pool, a_texture_path); 

  auto vulkan_device = a_device.GetVulkanDevice();
  CreateVulkanTextureImageView(*vulkan_device); 
}

TextureResources::~TextureResources()
{
}

int8_t TextureResources::CreateVulkanTextureImage(Device const &a_device, 
    CommandPool const &a_command_pool, std::string const &a_texture_path)
{
  auto vulkan_device = a_device.GetVulkanDevice();

  int32_t texture_width;
  int32_t texture_height;
  int32_t texture_channels;
  std::shared_ptr<stbi_uc> pixels(
      stbi_load(a_texture_path.c_str(), &texture_width, &texture_height,
        &texture_channels, STBI_rgb_alpha),
      [](stbi_uc *a_pixels) {
        stbi_image_free(a_pixels);
        std::cout << "DEBUG: stbi texture deleated." << std::endl;
      });

  VkDeviceSize image_size = texture_width * texture_height * 4;

  if (pixels == nullptr) {
    std::cerr << "Failed to read texture image '" << a_texture_path << "'." 
      << std::endl;
    return -1;
  }

  VkBufferCreateInfo buffer_info = {};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = image_size;
  buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  
  std::shared_ptr<VkBuffer> staging_buffer(new VkBuffer, 
      [vulkan_device](VkBuffer *a_staging_buffer) {
        vkDestroyBuffer(*vulkan_device, *a_staging_buffer, nullptr);
        std::cout << "DEBUG: Texture staging buffer deleted." << std::endl;
      });

  int32_t res = vkCreateBuffer(*vulkan_device, &buffer_info, nullptr, 
      &(*staging_buffer));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create texture staging buffer." << std::endl;
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

  VkMemoryAllocateInfo staging_allocate_info = {};
  staging_allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  staging_allocate_info.allocationSize = staging_memory_requirements.size;
  staging_allocate_info.memoryTypeIndex = staging_memory_type_index;

  std::shared_ptr<VkDeviceMemory> staging_buffer_memory(new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_staging_buffer_memory) {
        vkFreeMemory(*vulkan_device, *a_staging_buffer_memory, nullptr);
        std::cout << "DEBUG: Texture staging buffer memory freed." << std::endl;
      });

  res = vkAllocateMemory(*vulkan_device, &staging_allocate_info, nullptr, 
      &(*staging_buffer_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate texture staging buffer memory." 
      << std::endl;
    return -1;
  }

  vkBindBufferMemory(*vulkan_device, *staging_buffer, *staging_buffer_memory, 
      0);

  // TODO: Free 'data' later??
  void *data;
  vkMapMemory(*vulkan_device, *staging_buffer_memory, 0, image_size, 0, &data);
  memcpy(data, &(*pixels), static_cast<size_t>(image_size));
  vkUnmapMemory(*vulkan_device, *staging_buffer_memory);


  VkImageCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  create_info.imageType = VK_IMAGE_TYPE_2D;
  create_info.extent.width = texture_width;
  create_info.extent.height = texture_height;
  create_info.extent.depth = 1;
  create_info.mipLevels = 1;
  create_info.arrayLayers = 1;
  create_info.format = VK_FORMAT_R8G8B8A8_UNORM;
  create_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  create_info.initialLayout = VK_IMAGE_LAYOUT_PREINITIALIZED;
  create_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT 
    | VK_IMAGE_USAGE_SAMPLED_BIT;
  create_info.samples = VK_SAMPLE_COUNT_1_BIT;
  create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  m_vulkan_texture_image = std::shared_ptr<VkImage>(new VkImage, 
      [vulkan_device](VkImage *a_vulkan_texture_image) {
        vkDestroyImage(*vulkan_device, *a_vulkan_texture_image, nullptr);
        std::cout << "DEBUG: Texture image deleted." << std::endl;
      });

  res = vkCreateImage(*vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_texture_image));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create texture image." << std::endl;
    return -1;
  }


  VkMemoryPropertyFlags memory_properties = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

  VkMemoryRequirements memory_requirements;
  vkGetImageMemoryRequirements(*vulkan_device, *m_vulkan_texture_image,
      &memory_requirements);

  bool has_memory_type = a_device.HasMemoryType(memory_requirements,
      memory_properties);
  if (!has_memory_type) {
    std::cerr << "Failed to find suitable memory type for texture image." 
      << std::endl;
    return -1;
  }

  uint32_t memory_type_index = a_device.FindMemoryTypeIndex(
      memory_requirements, memory_properties);

  VkMemoryAllocateInfo allocate_info = {};
  allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocate_info.allocationSize = memory_requirements.size;
  allocate_info.memoryTypeIndex = memory_type_index;

  m_vulkan_texture_image_memory = std::shared_ptr<VkDeviceMemory>(
    new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_vulkan_texture_image_memory) {
        vkFreeMemory(*vulkan_device, *a_vulkan_texture_image_memory, nullptr);
        std::cout << "DEBUG: Depth image memory freed." << std::endl;
      });

  res = vkAllocateMemory(*vulkan_device, &allocate_info, nullptr, 
      &(*m_vulkan_texture_image_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate texture image memory." << std::endl;
    return -1;
  }

  vkBindImageMemory(*vulkan_device, *m_vulkan_texture_image,
      *m_vulkan_texture_image_memory, 0);
 
  TransitionImageLayout(a_device, a_command_pool, 
      VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
      static_cast<VkAccessFlagBits>(0), VK_ACCESS_TRANSFER_WRITE_BIT,
      VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);

  VkCommandBuffer command_buffer = 
    a_command_pool.BeginSingleTimeCommands(a_device);

  VkBufferImageCopy region = {};
  region.bufferOffset = 0;
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;
  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  region.imageSubresource.mipLevel = 0;
  region.imageSubresource.baseArrayLayer = 0;
  region.imageSubresource.layerCount = 1;
  region.imageOffset = {0, 0, 0};
  region.imageExtent = {static_cast<uint32_t>(texture_width), 
    static_cast<uint32_t>(texture_height), 1};

  vkCmdCopyBufferToImage(command_buffer, *staging_buffer, 
      *m_vulkan_texture_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, 
      &region);

  a_command_pool.EndSingleTimeCommands(a_device, command_buffer);

  TransitionImageLayout(a_device, a_command_pool,
      VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_ACCESS_TRANSFER_WRITE_BIT,
      VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
      VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

  return 0;
}

int8_t TextureResources::CreateVulkanTextureImageView(
    VkDevice const &a_vulkan_device)
{
  VkImageViewCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  create_info.image = *m_vulkan_texture_image;
  create_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  create_info.format = VK_FORMAT_R8G8B8A8_UNORM;
  create_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  create_info.subresourceRange.baseMipLevel = 0;
  create_info.subresourceRange.levelCount = 1;
  create_info.subresourceRange.baseArrayLayer = 0;
  create_info.subresourceRange.layerCount = 1;

  m_vulkan_texture_image_view = std::shared_ptr<VkImageView>(new VkImageView, 
      [a_vulkan_device](VkImageView *a_vulkan_texture_image_view) {
        vkDestroyImageView(a_vulkan_device, *a_vulkan_texture_image_view, 
            nullptr);
        std::cout << "DEBUG: Texture image view deleted." << std::endl;
      });

  int32_t res = vkCreateImageView(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_texture_image_view));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create texture image view." << std::endl;
    return -1;
  }

  return 0;
}

std::shared_ptr<VkImageView> TextureResources::GetVulkanTextureImageView() const
{
  return m_vulkan_texture_image_view;
}

int8_t TextureResources::TransitionImageLayout(Device const &a_device,
    CommandPool const &a_command_pool, VkImageLayout a_old_layout,
    VkImageLayout a_new_layout, VkAccessFlagBits a_src_access_mask, 
    VkAccessFlagBits a_dst_access_mask, VkPipelineStageFlags a_src_stage,
    VkPipelineStageFlags a_dst_stage)
{
  VkCommandBuffer command_buffer = 
    a_command_pool.BeginSingleTimeCommands(a_device);

  VkImageMemoryBarrier barrier = {};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.oldLayout = a_old_layout;
  barrier.newLayout = a_new_layout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = *m_vulkan_texture_image;
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;

  barrier.subresourceRange.baseMipLevel = 0;
  barrier.subresourceRange.levelCount = 1;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = 1;

  barrier.srcAccessMask = a_src_access_mask;
  barrier.dstAccessMask = a_dst_access_mask;

  vkCmdPipelineBarrier(command_buffer, a_src_stage, a_dst_stage, 0, 0, nullptr,
      0, nullptr, 1, &barrier);

  a_command_pool.EndSingleTimeCommands(a_device, command_buffer);
  
  return 0;
}
