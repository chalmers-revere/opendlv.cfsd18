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
#include <limits>

#include "device.hpp"
#include "surface.hpp"
#include "swapchain.hpp"

Swapchain::Swapchain(Surface const &a_surface, Device const &a_device, 
    uint32_t a_width, uint32_t a_height):
  m_vulkan_swapchain(),
  m_vulkan_swapchain_images(),
  m_swapchain_extent(),
  m_swapchain_image_format()
{
  auto vulkan_surface = a_surface.GetVulkanSurface();
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_physical_device = a_device.GetVulkanPhysicalDevice();

  int32_t graphics_queue_family = a_device.GetVulkanGraphicsQueueFamily();
  int32_t present_queue_family = a_device.GetVulkanPresentQueueFamily();

  CreateVulkanSwapchain(*vulkan_surface, *vulkan_device, 
      vulkan_physical_device, a_width, a_height, graphics_queue_family,
      present_queue_family);
}

Swapchain::~Swapchain()
{
}

// TODO: Split the below method into serveral
int8_t Swapchain::CreateVulkanSwapchain(
    VkSurfaceKHR const &a_vulkan_surface, VkDevice const &a_vulkan_device,
    VkPhysicalDevice a_vulkan_physical_device, uint32_t a_width, 
    uint32_t a_height, uint32_t a_graphics_queue_family, 
    uint32_t a_present_queue_family)
{
  std::vector<VkSurfaceFormatKHR> formats;

  uint32_t format_count;
  vkGetPhysicalDeviceSurfaceFormatsKHR(a_vulkan_physical_device, 
      a_vulkan_surface, &format_count, nullptr);

  if (format_count != 0) {
    formats.resize(format_count);
    vkGetPhysicalDeviceSurfaceFormatsKHR(a_vulkan_physical_device, 
        a_vulkan_surface, &format_count, formats.data());
  }

  bool format_set = false;
  VkSurfaceFormatKHR surface_format;
  if (formats.size() == 1 && formats[0].format == VK_FORMAT_UNDEFINED) {
    surface_format = {VK_FORMAT_B8G8R8A8_UNORM, 
        VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
    format_set = true;
  }

  if (!format_set) {
    for (auto const &format : formats) {
      if (format.format == VK_FORMAT_B8G8R8A8_UNORM 
          && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
        surface_format = format;
        format_set = true;
        break;
      }
    }
  }

  if (!format_set) {
    surface_format = formats[0];
  }


  std::vector<VkPresentModeKHR> present_modes;

  uint32_t present_mode_count;
  vkGetPhysicalDeviceSurfacePresentModesKHR(a_vulkan_physical_device, 
      a_vulkan_surface, &present_mode_count, nullptr);

  if (present_mode_count != 0) {
    present_modes.resize(present_mode_count);
    vkGetPhysicalDeviceSurfacePresentModesKHR(a_vulkan_physical_device, 
        a_vulkan_surface, &present_mode_count, present_modes.data());
  }

  bool present_mode_set = false;
  VkPresentModeKHR present_mode;
  for (auto const &mode : present_modes) {
    if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
      present_mode = mode;
      present_mode_set = true;
      break;
    }
  }

  if (!present_mode_set) {
    present_mode = VK_PRESENT_MODE_FIFO_KHR;
  }


  VkSurfaceCapabilitiesKHR capabilities;

  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(a_vulkan_physical_device, 
      a_vulkan_surface, &capabilities);

  VkExtent2D extent;
  if (capabilities.currentExtent.width != 
      std::numeric_limits<uint32_t>::max()) {
    extent = capabilities.currentExtent;
  } else {
    extent.width = std::max(capabilities.minImageExtent.width, 
        std::min(capabilities.maxImageExtent.width, a_width));
    extent.height = std::max(capabilities.minImageExtent.height, 
        std::min(capabilities.maxImageExtent.height, a_height));
  }


  uint32_t image_count = capabilities.minImageCount + 1;
  if (capabilities.maxImageCount > 0 
      && image_count > capabilities.maxImageCount) {
    image_count = capabilities.maxImageCount;
  }
  

  VkSwapchainCreateInfoKHR create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  create_info.surface = a_vulkan_surface;
  create_info.minImageCount = image_count;
  create_info.imageFormat = surface_format.format;
  create_info.imageColorSpace = surface_format.colorSpace;
  create_info.imageExtent = extent;
  create_info.imageArrayLayers = 1;
  create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
  

  if (a_graphics_queue_family != a_present_queue_family) {
    uint32_t queue_family_indices[] = {a_graphics_queue_family,
      a_present_queue_family};

    create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
    create_info.queueFamilyIndexCount = 2;
    create_info.pQueueFamilyIndices = queue_family_indices;
  } else {
    create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  }
  

  create_info.preTransform = capabilities.currentTransform;
  create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  create_info.presentMode = present_mode;
  create_info.clipped = VK_TRUE;
  //create_info.oldSwapchain = *m_vulkan_swapchain;
  

  m_vulkan_swapchain = std::shared_ptr<VkSwapchainKHR>(new VkSwapchainKHR, 
      [a_vulkan_device](VkSwapchainKHR *a_vulkan_swapchain) {
        vkDestroySwapchainKHR(a_vulkan_device, *a_vulkan_swapchain, nullptr);
        std::cout << "DEBUG: Swapchain deleted." << std::endl;
      });
  

  int32_t res = vkCreateSwapchainKHR(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_swapchain));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create swapchain." << std::endl;
    return -1;
  }
  

  vkGetSwapchainImagesKHR(a_vulkan_device, *m_vulkan_swapchain, &image_count,
      nullptr);
  
  m_vulkan_swapchain_images = std::shared_ptr<std::vector<VkImage>>(
      new std::vector<VkImage>);
  (*m_vulkan_swapchain_images).resize(image_count);
  vkGetSwapchainImagesKHR(a_vulkan_device, *m_vulkan_swapchain, &image_count,
      (*m_vulkan_swapchain_images).data());

  m_swapchain_extent = extent;
  m_swapchain_image_format = surface_format.format;

  return 0;
}

std::shared_ptr<VkSwapchainKHR> Swapchain::GetVulkanSwapchain() const
{
  return m_vulkan_swapchain;
}

std::shared_ptr<std::vector<VkImage>> Swapchain::GetVulkanSwapchainImages() 
  const
{
  return m_vulkan_swapchain_images;
}

VkFormat Swapchain::GetVulkanSwapchainImageFormat() const
{
  return m_swapchain_image_format;
}

VkExtent2D Swapchain::GetVulkanSwapchainExtent() const
{
  return m_swapchain_extent;
}
