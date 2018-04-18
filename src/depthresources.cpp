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

#include "commandpool.hpp"
#include "depthresources.hpp"
#include "device.hpp"
#include "renderpass.hpp"
#include "swapchain.hpp"

DepthResources::DepthResources(Device const &a_device, 
    Swapchain const &a_swapchain, RenderPass const &a_render_pass, 
    CommandPool const &a_command_pool): 
  m_vulkan_depth_image_memory(),
  m_vulkan_depth_image(),
  m_vulkan_depth_image_view()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_swapchain_extent = a_swapchain.GetVulkanSwapchainExtent(); 
  auto vulkan_depth_format = a_render_pass.GetVulkanDepthFormat();
  
  CreateVulkanDepthImage(*vulkan_device, vulkan_swapchain_extent,
      vulkan_depth_format); 
  CreateVulkanDepthImageMemory(a_device);
  CreateVulkanDepthImageView(*vulkan_device, vulkan_depth_format); 

  TransitionImageLayout(a_device, a_command_pool, vulkan_depth_format);
}

DepthResources::~DepthResources()
{
}

int8_t DepthResources::CreateVulkanDepthImage(VkDevice const &a_vulkan_device, 
    VkExtent2D a_vulkan_swapchain_extent, VkFormat a_vulkan_depth_format)
{
  VkImageCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  create_info.imageType = VK_IMAGE_TYPE_2D;
  create_info.extent.width = a_vulkan_swapchain_extent.width;
  create_info.extent.height = a_vulkan_swapchain_extent.height;
  create_info.extent.depth = 1;
  create_info.mipLevels = 1;
  create_info.arrayLayers = 1;
  create_info.format = a_vulkan_depth_format;
  create_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  create_info.initialLayout = VK_IMAGE_LAYOUT_PREINITIALIZED;
  create_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  create_info.samples = VK_SAMPLE_COUNT_1_BIT;
  create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  m_vulkan_depth_image = std::shared_ptr<VkImage>(new VkImage, 
      [a_vulkan_device](VkImage *a_vulkan_depth_image) {
        vkDestroyImage(a_vulkan_device, *a_vulkan_depth_image, nullptr);
        std::cout << "DEBUG: Depth image deleted." << std::endl;
      });

  int32_t res = vkCreateImage(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_depth_image));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create depth image." << std::endl;
    return -1;
  }

  return 0;
}

int8_t DepthResources::CreateVulkanDepthImageMemory(Device const &a_device)
{
  auto vulkan_device = a_device.GetVulkanDevice();

  VkMemoryPropertyFlags memory_properties = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

  VkMemoryRequirements memory_requirements;
  vkGetImageMemoryRequirements(*vulkan_device, *m_vulkan_depth_image,
      &memory_requirements);

  bool has_memory_type = a_device.HasMemoryType(memory_requirements,
      memory_properties);
  if (!has_memory_type) {
    std::cerr << "Failed to find suitable memory type." << std::endl;
    return -1;
  }

  uint32_t memory_type_index = a_device.FindMemoryTypeIndex(
      memory_requirements, memory_properties);

  VkMemoryAllocateInfo allocate_info = {};
  allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocate_info.allocationSize = memory_requirements.size;
  allocate_info.memoryTypeIndex = memory_type_index;

  m_vulkan_depth_image_memory = std::shared_ptr<VkDeviceMemory>(
    new VkDeviceMemory, 
      [vulkan_device](VkDeviceMemory *a_vulkan_depth_image_memory) {
        vkFreeMemory(*vulkan_device, *a_vulkan_depth_image_memory, nullptr);
        std::cout << "DEBUG: Depth image memory freed." << std::endl;
      });

  int32_t res = vkAllocateMemory(*vulkan_device, &allocate_info, nullptr, 
      &(*m_vulkan_depth_image_memory));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to allocate depth image memory." << std::endl;
    return -1;
  }

  vkBindImageMemory(*vulkan_device, *m_vulkan_depth_image,
      *m_vulkan_depth_image_memory, 0);

  return 0;
}

int8_t DepthResources::CreateVulkanDepthImageView(
    VkDevice const &a_vulkan_device, VkFormat a_vulkan_depth_format)
{
  VkImageViewCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  create_info.image = *m_vulkan_depth_image;
  create_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  create_info.format = a_vulkan_depth_format;
  create_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  create_info.subresourceRange.baseMipLevel = 0;
  create_info.subresourceRange.levelCount = 1;
  create_info.subresourceRange.baseArrayLayer = 0;
  create_info.subresourceRange.layerCount = 1;

  m_vulkan_depth_image_view = std::shared_ptr<VkImageView>(new VkImageView, 
      [a_vulkan_device](VkImageView *a_vulkan_depth_image_view) {
        vkDestroyImageView(a_vulkan_device, *a_vulkan_depth_image_view, 
            nullptr);
        std::cout << "DEBUG: Depth image view deleted." << std::endl;
      });

  int32_t res = vkCreateImageView(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_depth_image_view));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create depth image view." << std::endl;
    return -1;
  }

  return 0;
}

std::shared_ptr<VkImageView> DepthResources::GetVulkanDepthImageView() const
{
  return m_vulkan_depth_image_view;
}

int8_t DepthResources::TransitionImageLayout(Device const &a_device,
    CommandPool const &a_command_pool, VkFormat a_vulkan_depth_format)
{
  VkCommandBuffer command_buffer = a_command_pool.BeginSingleTimeCommands(
      a_device);

  bool has_stencil_component = 
    (a_vulkan_depth_format == VK_FORMAT_D32_SFLOAT_S8_UINT || 
     a_vulkan_depth_format == VK_FORMAT_D24_UNORM_S8_UINT);

  VkImageMemoryBarrier barrier = {};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  barrier.newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = *m_vulkan_depth_image;
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

  if (has_stencil_component) {
    barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
  }

  barrier.subresourceRange.baseMipLevel = 0;
  barrier.subresourceRange.levelCount = 1;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = 1;

  barrier.srcAccessMask = 0;
  barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

  vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
      VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT, 0, 0, nullptr, 0, nullptr, 1, 
      &barrier);

  a_command_pool.EndSingleTimeCommands(a_device, command_buffer);
  
  return 0;
}
