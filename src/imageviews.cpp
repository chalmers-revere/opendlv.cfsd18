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

#include "device.hpp"
#include "swapchain.hpp"
#include "imageviews.hpp"

ImageViews::ImageViews(Device const &a_device, Swapchain const &a_swapchain): 
  m_vulkan_image_views()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_swapchain_images = a_swapchain.GetVulkanSwapchainImages();
  auto vulkan_swapchain_image_format = 
    a_swapchain.GetVulkanSwapchainImageFormat();

  CreateVulkanImageViews(*vulkan_device, vulkan_swapchain_images,
      vulkan_swapchain_image_format);
}

ImageViews::~ImageViews()
{
}

int8_t ImageViews::CreateVulkanImageViews(VkDevice const &a_vulkan_device,
    std::shared_ptr<std::vector<VkImage>> a_vulkan_swapchain_images,
    VkFormat a_vulkan_swapchain_image_format)
{
  m_vulkan_image_views.resize((*a_vulkan_swapchain_images).size());

  for (uint32_t i = 0; i < (*a_vulkan_swapchain_images).size(); i++) {

    m_vulkan_image_views[i] = std::shared_ptr<VkImageView>(new VkImageView, 
        [a_vulkan_device, i](VkImageView *a_vulkan_image_view) {
          vkDestroyImageView(a_vulkan_device, *a_vulkan_image_view, nullptr);
          std::cout << "DEBUG: Image view " << i << " deleted." << std::endl;
        });

    VkImageViewCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    create_info.image = (*a_vulkan_swapchain_images).at(i);
    create_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    create_info.format = a_vulkan_swapchain_image_format;
    create_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    create_info.subresourceRange.baseMipLevel = 0;
    create_info.subresourceRange.levelCount = 1;
    create_info.subresourceRange.baseArrayLayer = 0;
    create_info.subresourceRange.layerCount = 1;

    int32_t res = vkCreateImageView(a_vulkan_device, &create_info, nullptr, 
        &(*m_vulkan_image_views[i]));
    if (res != VK_SUCCESS) {
      std::cerr << "Failed to create image view " << i << "." << std::endl;
      return -1;
    }
  }

  return 0;
}

std::shared_ptr<VkImageView> ImageViews::GetVulkanImageView(uint32_t a_index) 
  const
{
  return m_vulkan_image_views[a_index];
}

uint32_t ImageViews::GetImageViewCount() const
{
  return m_vulkan_image_views.size();
}
