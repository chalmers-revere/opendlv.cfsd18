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

#ifndef SWAPCHAIN_HPP
#define SWAPCHAIN_HPP

#include <vulkan/vulkan.h>

class Device;
class Surface;

/**
 * @brief A wrapper for a Vulkan swapchain
 */
class Swapchain {
  public:
    Swapchain(Surface const &, Device const &, uint32_t, 
        uint32_t);
    Swapchain(Swapchain const &) = delete;
    Swapchain &operator=(Swapchain const &) = delete;
    virtual ~Swapchain();
    std::shared_ptr<VkSwapchainKHR> GetVulkanSwapchain() const;
    std::shared_ptr<std::vector<VkImage>> GetVulkanSwapchainImages() const;
    VkExtent2D GetVulkanSwapchainExtent() const;
    VkFormat GetVulkanSwapchainImageFormat() const;
  
  private:
    int8_t CreateVulkanSwapchain(VkSurfaceKHR const &, VkDevice const &,
        VkPhysicalDevice, uint32_t, uint32_t, uint32_t, uint32_t);

    std::shared_ptr<VkSwapchainKHR> m_vulkan_swapchain;
    std::shared_ptr<std::vector<VkImage>> m_vulkan_swapchain_images;
    VkExtent2D m_swapchain_extent;
    VkFormat m_swapchain_image_format;
};

#endif
